/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "apis.h"
//
#include "auton_globals.h"
#include "autos.h"
#include "blazing/utils.hpp"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/config.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"
#include "units/Angle.hpp"
#include <iostream>

// do not do anything outside here!

namespace qual_match_first {

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

    auto make_machloader_point = [&](units::V2FPosition target,
                                     Length distance) -> units::V2FPosition {
        auto target_angle = target.angleTo(RobotGetPose());

        return target + distance * (RobotGetPose() - target).normalize();
    };

    auto start_time = now();

    units::V2FPosition target_point;

    intake::in();

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    Length match1 = normal_match;
    Length match2 = normal_match;
    Length match3 = -normal_match;
    Length match4 = -normal_match;

    units::V2Position centerBallOne = { -24_in, 24_in };

    units::V2Position centerTopGoalFirst = { -7.9_in, 7.4_in };
    // units::V2Position centerBottomGoalFirst = { -10.3_in, -11_in };
    units::V2Position centerBottomGoalFirst = { -12.4_in, -13.4_in };

    alliance_t opposite_alliance =
      auto_alliance == alliance_t::red ? alliance_t::blue : alliance_t::red;

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    double angle = bl ? 90 : 270;

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

        // make sure we are matchloading
        matchloader::down();

        Time motion_start_time = now();

        mb.moveTo(func)
            // if it takes longer it most likely got stuck
            .timeout(1.5_sec)
            .drive_vel_accelSlew(110_inps2)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_vel_mp_setMaxAccel(70_inps2) |
          async;

        Length matchload_start_distance = 13_in;

        auto custom_exit_condition = [&] -> bool {
            auto curr_pose = RobotGetPose();
            auto error = (target_Point - curr_pose);
            auto local_error = error.rotatedBy(-curr_pose.orientation);

            bool close = error.magnitude() <
                         // trigger only if closes to the matchloader
                         matchload_start_distance + 5_in;

            bool forwards_close =
              units::abs(local_error.x) < matchload_start_distance;

            // use forwards error and
            return close && forwards_close;
        };

        auto wait_result = async.waitOr(custom_exit_condition, 3_sec);

        if (wait_result == blazing::AsyncExecutorBase::motionFinished ||
            wait_result == blazing::AsyncExecutorBase::timeoutFinished) {
            // custom condition did not trigger, meaning we got stuck or
            // something else went wrong. Don't wait just exit
            async.exitAll();
        } else {
            // got to matcloader successfully, start matchloading
            async.exitAll();
            // passive voltage forwards since motion might oscilate
            drivetrain.moveTank(0.13_volt, 0.13_volt);
            pros::delay(to_msec(matchload_time));
        }
    };

    auto score_long_goal = [](double sign_x,
                              double sign_y,
                              Time score_time,
                              bool with_swing = false) {
        // turn to goal, reversed
        Length long_goal = 47.05_in;

        auto target_backwards_heading = sign_x == -1 ? 0_stDeg : 180_stDeg;
        auto target_forwards_heading = sign_x == -1 ? 180_stDeg : 0_stDeg;

        units::Pose target_pose = { 24_in * sign_x,
                                    long_goal * sign_y,
                                    target_backwards_heading };

        auto exit_condition = [&] -> bool {
            auto curr_pose = RobotGetPose();
            bool x_close = units::abs(curr_pose.x) >= 27_in &&
                           units::abs(curr_pose.x) <= 29.5_in;
            bool y_close = units::abs(curr_pose.y) >= 43_in &&
                           units::abs(curr_pose.y) <= 51_in;
            //
            bool theta_close =
              units::abs(angleError(target_forwards_heading,
                                    curr_pose.orientation)) <= 25_stDeg;

            return x_close && y_close && theta_close;
        };

        // turn towards 24, settle at 48
        if (!with_swing) {
            mb.moveTo(target_pose)
                .drive_vel_mp_setMaxAccel(110_inps2)
                .only_x(true, 28_in)
                .closeThreshold(10_in)
                .timeout(2_sec)
                .reverse() |
              chain;
        } else {
            mb.moveTo(17_in * sign_x, 54.5_in * sign_y)
                .reverse()
                .drive_vel_minVel(50_inps)
                .setChainTime(0_sec) |
              chain;

            // mb.turnTo(21.8_in, 47_in)
            mb.turnTo(target_backwards_heading)
                .reverse()
                .direction(AngularDirection::RIGHT)
                .radius(-10.5_in / 2)
                .timeout(2.6_sec) |
              chain;
        }

        chain.waitOr(exit_condition);

        // regardless of getting stuck or not we perform the same action

        intake::score_long();
        // let move to point settle a bit
        pros::delay(100);
        chain.exitAll();
        // queue aligning motion
        mb.turnTo(target_forwards_heading).radius(-4.0_in) | chain;

        pros::delay(units::max(to_msec(score_time) - 100, 0));
        chain.exitAll();
    };

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(inactive);

    RobotSetPose(-48.2, 16.3 * l, angle);

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    mb.moveTo(-48, (match1 - 0.1_in) * l) | run;

    // its a run here, so we can do these things
    intake::in();
    // turn to and go to matchloader
    matchloader::down();
    mb.turnTo(make_matchloader_point(-1, l)).turn_toleranceDuration(25_msec) |
      run;

    matchload(-1, l, 0.30_sec);
    score_long_goal(-1, l, 1_sec);

    matchloader::up();

    intake::in();

    if (bl) {
        mb.moveTo(-23.5, 23.4 * l).drive_maxVolt(0.45_volt) | async;

        async.waitUntil(closeEnough({ -25.5_in, 23.4_in * l }, 10_in));
        matchloader::down();

        async.wait();

        mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y).reverse() | chain;

        mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
            .reverse()
            .k_lat(0.3)
            .drive_maxVolt(0.5_volt)
            .executeBeforeMotion([] {
                pros::Task([] {
                    matchloader::down();
                    pros::delay(220);
                    matchloader::up();
                });
            }) |
          chain;

        chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 5.5_in));
        intake::score_middle();
        pros::delay(1500);
        chain.exitAll();
    } else {
        mb.moveTo(-23.6, 23.6 * l) | async;

        async.waitUntil(closeEnough({ -23.6_in, 23.6_in * l }, 10_in));
        matchloader::down();

        async.wait();

        mb.turnTo(0, 0) | run;

        mb.moveTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y)
            .k_lat(0.3)
            .drive_maxVolt(0.3_volt)
            .executeBeforeMotion([] {
                // pros::Task([] {
                // pros::delay(200);
                matchloader::up();
                // });
            }) |
          chain;
        mb.turnTo(0, 0) | run;

        chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 5.5_in));
        // score but with slightly less power
        intake::score_bottom(-0.3);
        pros::delay(1400);
        chain.exitAll();
    }
}

} // namespace qual_match_first
