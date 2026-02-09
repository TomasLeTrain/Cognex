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

namespace states_skills {

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
                              bool with_swing = false) {
        // turn to goal, reversed
        Length long_goal = 47.05_in;

        auto target_heading = sign_x == -1 ? 0_stDeg : 180_stDeg;
        auto target_forwards_heading = sign_x == -1 ? 180_stDeg : 0_stDeg;

        units::Pose target_pose = { 24_in * sign_x,
                                    long_goal * sign_y,
                                    target_heading };

        // turn towards 24, settle at 48
        if (with_swing) {
            // do the swing
            mb.turnTo(target_forwards_heading).radius(-10.5 / 2) | async;
        } else {
            mb.moveTo(target_pose)
                .drive_vel_mp_setMaxAccel(110_inps2)
                .only_x(true, 28_in)
                .closeThreshold(10_in)
                .timeout(2_sec)
                .reverse() |
              async;
        }

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

    /* START AUTON */

    RobotSetPose(-43.3, 0.0, 180);

    intake::in();

    // swing back a bit
    drivetrain.moveTank(-0_volt, -1_volt);
    pros::delay(300);
    mb.moveTo(-40, -19).reverse() | run;

    mb.turnTo(-30.414, -17.062) | run;
    mb.moveTo(-30.414, -17.062) | run;
    mb.moveTo(-13.097, -13.691).closeThreshold(4_in) | run;

    // TODO: align to the goal and score

    drivetrain.moveTank(-1_volt, -1_volt);
    pros::delay(200);

    mb.turnTo(-43.02, 44.349) | run;
    mb.moveTo(-43.02, 44.349) | run;

    matchload(-1, 1, 1.5_sec);

    // go away from matchloader
    mb.moveTo(-30.692, 59.614) | run;

    // prepare for swing
    mb.moveTo(14.375, 56.613) | run;

    score_long_goal(1, 1, 2_sec, true);

    matchload(1, 1, 1.5_sec);

    score_long_goal(1, 1, 2_sec);

    // sprint straight towards second park

    mb.moveTo(41.142, 0) | run;
    mb.turnTo(0) | run;

    // TODO: perform getting balls from park

    // swing back a bit
    drivetrain.moveTank(-1.0_volt, -0.5_volt);
    pros::delay(170);
    mb.turnTo(29.545, -16.952) | run;
    mb.moveTo(29.545, -16.952) | run;
    mb.turnTo(10.574, -10.401).reverse() | run;
    mb.moveTo(10.574, -10.401).reverse() | run;

    // TODO: align well and score

    mb.moveTo(44.745, -44.164) | run;
    matchload(1, 1, 1.5_sec);

    // go away from matchloader
    mb.moveTo(30.692, -59.614) | run;

    // prepare for swing
    mb.moveTo(-14.375, -56.613) | run;

    score_long_goal(-1, -1, 2_sec, true);

    matchload(-1, -1, 1.5_sec);

    score_long_goal(-1, -1, 2_sec);

    // finally park

    mb.moveTo(-66.246, -18.014) | run;
    mb.turnTo(90) | run;

    drivetrain.moveTank(0.5_volt, 0.5_volt);
    pros::delay(300);
    drivetrain.moveTank(0.0_volt, 0.0_volt);

	// cinema
}

} // namespace states_skills
