/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "apis.h"
//
#include "autos.h"
#include "blazing/utils.hpp"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/config.h"
#include "globals/device_globals.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include <iostream>
#include <tuple>

namespace awp {

void pre_auton() {
    // set the robot state to match expectations
    // done in case driver or such is run before auto
    wings::up();
    odom_retract::lowerOdom();
    matchloader::up();

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    intake::setAutonColorSort(false);
}

void run_auton() {
    // runs before anything else
    pre_auton();

    units::V2Position centerTopGoalFirst = { -9.5_in, 8.0_in };

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    auto make_matchloader_point = [](double sign_x,
                                     double sign_y) -> units::V2Position {
        Length normal_match = 46.7_in;
        return { 67.4_in * sign_x, normal_match * sign_y };
    };

    auto matchload = [make_matchloader_point](double sign_x,
                                              double sign_y,
                                              Time matchload_time) {
        auto make_machloader_pose = [](units::V2FPosition target,
                                       Length distance) -> units::Pose {
            // auto target_angle = target.angleTo(RobotGetPose());
            auto final_point =
              target + distance * (RobotGetPose() - target).normalize();

            return units::Pose { final_point, final_point.angleTo(target) };
        };

        Length normal_match = 46.7_in;
        units::V2Position target_Point = make_matchloader_point(sign_x, sign_y);
        Length target_dist = 11_in;

        auto func = [&] -> units::Pose {
            return make_machloader_pose(target_Point, target_dist);
        };

        // make sure we are matchloading and intaking?
        matchloader::down();
        intake::in();

        Time motion_start_time = now();

        mb.moveTo(func)
            // if it takes longer it most likely got stuck
            .timeout(1.5_sec)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_vel_mp_setMaxAccel(70_inps2) |
          async;

        Length matchload_start_distance = 13_in;

        auto custom_exit_condition = [&] -> bool {
            // use forwards error and
            return (target_Point - RobotGetPose()).magnitude() <
                     // trigger only if closes to the matchloader
                     matchload_start_distance + 5_in &&
                   units::abs((target_Point - RobotGetPose())
                                .rotatedBy(-RobotGetPose().orientation)
                                .x) < matchload_start_distance;
        };

        bool motion_finished = false;

        while (true) {
            bool exit_now = custom_exit_condition();
            motion_finished = async.numQueuedMotions() == 0;

            if (motion_finished || exit_now) break;
            pros::delay(10);
        }

        if (motion_finished) {
            // motion finished before matchload start time, we likely got stuck
            // and should stop any more matchloading time
            async.exitAll();
        } else {
            // got to matcloader successfully, start matchloading
            pros::delay(to_msec(matchload_time));
            async.exitAll();
        }
    };

    auto score_long_goal = [](double sign_x,
                              double sign_y,
                              Time score_time,
                              bool from_matchloader = false) {
        // turn to goal, reversed
        Length long_goal = 47.05_in;

        auto target_heading = sign_x == -1 ? 0_stDeg : 180_stDeg;
        auto target_forwards_heading = sign_x == -1 ? 180_stDeg : 0_stDeg;

        units::Pose target_pose = { 24_in * sign_x,
                                    long_goal * sign_y,
                                    target_heading };

        // turn towards 24, settle at 48
        mb.moveTo(target_pose)
            .drive_vel_mp_setMaxAccel(110_inps2)
            .only_x(true, 28_in)
            .closeThreshold(10_in)
            .timeout(2_sec)
            .reverse() |
          async;

        async.waitUntil([&] -> bool {
            // use forwards error and

            auto curr_pose = RobotGetPose();
            bool x_close = units::abs(curr_pose.x) >= 27_in &&
                           units::abs(curr_pose.x) <= 29.5_in;
            bool y_close = units::abs(curr_pose.y) >= 43_in &&
                           units::abs(curr_pose.y) <= 51_in;
            //
            bool theta_close =
              units::abs(angleError(target_forwards_heading,
                                    curr_pose.orientation)) < 25_stDeg;

            return x_close && y_close && theta_close;
            // return x_close;
        });
        intake::score_long();
        // let move to point settle a bit
        pros::delay(100);
        async.exitAll();
        // queue aligning motion
        mb.turnTo(target_forwards_heading).radius(-4.0_in) | async;

        pros::delay(units::max(to_msec(score_time) - 100, 0));
        async.exitAll();
    };

    // start auton
    RobotSetPose(-46.57, -14, 90);

    intake::in();

    bool pushing = false;

    if (pushing) mb.moveTo(-46.57, -4.7).timeout(1.2_sec) | chain;

    mb.moveTo(-46.376, -normal_match - 0.5_in)
        .only_y(true)
        .reverse()
        .drive_errorTolerance(1_in)
        .customAngularLinearFunc([](Angle angle) {
            return units::cos(angle);
        })
        .timeout(1.3_sec) |
      run;
    // chain.wait();

    // turn to and go to matchloader
    // matchloader::down();

    // mb.turnTo(make_matchloader_point(-1, -1))
    //     .turn_toleranceDuration(25_msec)
    //     .timeout(0.8_sec) |
    //   async;
    // async.wait();

    // pros::delay(50);
    matchload(-1, -1, 0.35_sec);
    score_long_goal(-1, -1, 1_sec, true);

    matchloader::up();

    // mb.turnTo(-1_tile, -1_tile).radius(-1.2) | chain;

    drivetrain.moveTank(1.0_volt, -0.8_volt);
    pros::delay(200);

    mb.moveTo(-1_tile, -1_tile)
        // .drive_minVolt(0.5_volt)
        .drive_vel_minVel(50_inps)
        .setChainTime(0_sec) |
      chain;

    mb.moveTo(-1_tile, 1_tile)
        // already going fast from previous motion, slew can be faster
        .drive_vel_accelSlew(300_inps)
        .setChainTime(0_sec) |
      chain;

    // mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y).reverse() | chain;

    mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
        .reverse()
        .closeThreshold(4_in)
      // .drive_vel_maxVel(40_inps)
      | chain;

    chain.waitUntil(closeEnough({ -1_tile, -1_tile }, 7_in));
    intake::in();
    chain.waitUntil(closeEnough({ -1_tile, 1_tile }, 10_in));
    matchloader::down();
    chain.waitUntil(closeEnough(centerTopGoalFirst, 4_in));
    // start scoring
    intake::score_middle();
    // give time for it to settle
    pros::delay(300);
    chain.exitAll();

    // align well
    mb.turnTo(135).radius(-4.0_in) | async;

    // drivetrain.moveTank(-0.1_volt, -0.1_volt);

    pros::delay(800);

    async.exitAll();

    intake::in();

    // turn to and go to matchloader
    mb.moveTo(-48_in, normal_match).drive_errorTolerance(1_in).only_y(true) |
      chain;
    chain.wait();

    // turn to and go to matchloader
    // mb.turnTo(make_matchloader_point(-1, 1)) | run;

    // pros::delay(50);
    matchload(-1, 1, 0.35_sec);
    score_long_goal(-1, 1, 1_sec, true);
}

} // namespace easier_awp
