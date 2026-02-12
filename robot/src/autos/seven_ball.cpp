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
#include "globals/device_globals.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"
#include "units/Angle.hpp"
#include <iostream>

// do not do anything outside here!

namespace seven_ball {

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

    // do whatever you want here
    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    Length match1 = normal_match;

    units::V2Position centerTopGoalFirst = { -7.9_in, 7.4_in };
    units::V2Position centerBottomGoalFirst = { -11_in, -11_in };

    bool winging = true;
    bool fast_wing = false;
    Voltage slow_wing_speed = 0.5_volt;

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

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


    double angle = bl ? 0 : 0;

    intake::in();

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::up();

    RobotSetPose(-47.2, 14.9 * l, angle);
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // only intake bottom balls to save time
    intake::in();

    mb.moveTo(-23.5, 21.4 * l).drive_maxVolt(0.45_volt) | chain;

    chain.waitUntil(closeEnough({ -25.5_in, 23.4_in * l }, 9.0_in));
    matchloader::down();

    chain.wait();

    mb.moveTo(-48, (match1)*l).reverse() | run;

    // its a run here, so we can do these things
    intake::in();

    // turn to and go to matchloader
    matchloader::down();
    mb.turnTo(make_matchloader_point(-1, l)).turn_toleranceDuration(25_msec) |
      run;

    matchload(-1, l, 0.30_sec);
    score_long_goal(-1, l, 2.0_sec);

    matchloader::up();

    if (winging) {
        if (bl) {
            mb.moveTo(-35.737, 37.1) | chain;
            mb.turnTo(0).reverse() | chain;
            wings::down();
            mb.boomerang(-9, 37, 0)
                .reverse()
                .drive_maxVolt(fast_wing ? 1.0_volt : slow_wing_speed)
                .drive_toleranceDuration(100_sec)
                .drive_largeToleranceDuration(100_sec)
                .timeout(100_sec) |
              chain;

        } else {
            mb.moveTo(-35.737, -36.7) | chain;
            mb.turnTo(0) | chain;
            wings::down();
            // mb.moveTo(-9, -37.2)
            //     .drive_maxVolt(fast_wing ? 1.0_volt : slow_wing_speed)
            //     .drive_toleranceDuration(100_sec)
            //     .drive_largeToleranceDuration(100_sec)
            //     .timeout(100_sec) |
            //   run;

            mb.boomerang(-8.7, -37.2, 0)
                .drive_maxVolt(fast_wing ? 1.0_volt : slow_wing_speed)
                .drive_toleranceDuration(100_sec)
                .drive_largeToleranceDuration(100_sec)
                .timeout(100_sec) |
              chain;
        }
    }

    // pros::delay(500);
    // // start_time = now();
    //
    // intake::set(intake::scoring_long_top_balls_outake_bottom);
    //
    // pros::delay(1700);
}

} // namespace seven_ball
