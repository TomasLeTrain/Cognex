/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "apis.h"
//
#include "autos.h"
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

namespace easier_awp {

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

    units::V2Position centerTopGoalFirst = { -8.2_in, 7.4_in };

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

        intake::in();
        mb.moveTo(func)
            .timeout(3_sec)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_vel_mp_setMaxAccel(70_inps2) |
          async;

        Length matchload_start_distance = 13_in;
        async.waitUntil([&] {
            // use forwards error and
            return (target_Point - RobotGetPose()).magnitude() <
                     // trigger only if closes to the matchloader
                     matchload_start_distance + 5_in &&
                   units::abs((target_Point - RobotGetPose())
                                .rotatedBy(-RobotGetPose().orientation)
                                .x) < matchload_start_distance;
        });
        // controller.rumble(".");
    };

    auto score_long_goal = [](double sign_x,
                              double sign_y,
                              Time score_time,
                              bool from_matchloader = false) {
        // turn to goal, reversed
        Length long_goal = 47.1_in;
        Length normal_match = 46.7_in;

        units::Pose target_pose = { 24_in * sign_x,
                                    long_goal * sign_y,
                                    sign_x == -1 ? 0_stDeg : 180_stDeg };

        units::Pose other_target_pose = { 24_in * sign_x,
                                          long_goal * sign_y,
                                          sign_x == -1 ? 0_stDeg : 180_stDeg };

        // turn towards 24, settle at 48
        mb.moveTo(target_pose)
            .drive_vel_mp_setMaxAccel(110_inps2)
            .only_x(true, 28_in)
            .closeThreshold(10_in)
            .timeout(2_sec)
            .reverse() |
          run;
        pros::delay(to_msec(score_time));
    };

    // start auton
    RobotSetPose(-46.57, -14, 90);

    intake::in();

    bool pushing = false;

    if (pushing) mb.moveTo(-46.57, -4.7).timeout(1.2_sec) | chain;

    mb.moveTo(-46.376, -normal_match - 0.5_in).reverse().timeout(1.3_sec) |
      chain;
    chain.wait();

    // turn to and go to matchloader
    matchloader::down();
    mb.turnTo(make_matchloader_point(-1, -1))
        .turn_toleranceDuration(25_msec)
        .timeout(0.8_sec) |
      async;
    async.wait();

    // pros::delay(50);
    matchload(-1, -1, 0.3_sec);
    score_long_goal(-1, -1, 1_sec, true);

    matchloader::up();

    mb.turnTo(-1_tile, -1_tile).radius(-1.2) | chain;

    mb.moveTo(-1_tile, -1_tile)
        .executeAfterMotion([] {
            intake::in();
        })
        .drive_minVolt(0.5_volt)
        .setChainTime(0_sec) |
      chain;

    mb.moveTo(-1_tile, 1_tile).executeAfterMotion([] {
        intake::in();
    }) |
      chain;

    mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y).reverse() | chain;

    mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
        .reverse()
        .k_lat(0.3)
        // .drive_maxVolt(0.5_volt)
        .drive_vel_maxVel(40_inps)
      // .drive_kp(linear_pid.get_kp() * 0.7)
      | chain;

    chain.waitUntil(closeEnough({ -1_tile, -1_tile }, 7_in));
    matchloader::down();
    pros::delay(500);
    matchloader::up();
    chain.waitUntil(closeEnough({ -1_tile, 1_tile }, 7_in));
    matchloader::down();
    chain.waitUntil(closeEnough({ -8_in, 8_in }, 5.5_in));
    intake::score_middle();
    chain.exitAll();
    drivetrain.moveTank(-0.1_volt, -0.1_volt);

    pros::delay(1100);
    intake::in();

    // turn to and go to matchloader
    mb.moveTo(-48_in, normal_match + 0.5_in).only_y(true) | chain;
    chain.wait();

    // turn to and go to matchloader
    mb.turnTo(make_matchloader_point(-1, 1)) | run;

    // pros::delay(50);
    matchload(-1, 1, 0.3_sec);
    score_long_goal(-1, 1, 1_sec, true);
}

} // namespace easier_awp
