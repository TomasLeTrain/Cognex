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

void matchload(double sign_x, double sign_y, Time matchload_time) {
    auto make_matchloader_point = [](double sign_x,
                                     double sign_y) -> units::V2Position {
        Length normal_match = 46.7_in;
        return { 67.4_in * sign_x, normal_match * sign_y };
    };

    auto make_machloader_pose = [](units::V2FPosition target,
                                   Length distance) -> units::Pose {
        // auto target_angle = target.angleTo(RobotGetPose());
        auto final_point =
          target + distance * (RobotGetPose() - target).normalize();

        return units::Pose { final_point, final_point.angleTo(target) };
    };

    Length normal_match = 46.7_in;
    units::V2Position target_Point = make_matchloader_point(sign_x, sign_y);

    // 11 is barely achievable - 0.1 less than achievable
    Length target_dist = 10.9_in;

    auto func = [&] -> units::Pose {
        return make_machloader_pose(target_Point, target_dist);
    };

    // make sure we are matchloading
    matchloader::down();

    mb.turnTo(func()) | run;

    mb.moveTo(func())
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
        drivetrain.moveTank(0.2_volt, 0.2_volt);
        pros::delay(to_msec(matchload_time));
    }
}

void score_long_goal(double sign_x, double sign_y, Time score_time) {
    // turn to goal, reversed
    Length long_goal = 47.05_in;

    auto target_backwards_heading = sign_x == -1 ? 0_stDeg : 180_stDeg;
    auto target_forwards_heading = sign_x == -1 ? 180_stDeg : 0_stDeg;
    auto boomerang_heading = sign_x == -1 ? 170_stDeg : 350_stDeg;

    units::Pose target_pose = { 24_in * sign_x,
                                long_goal * sign_y,
                                target_backwards_heading };

    auto exit_condition = [&] -> bool {
        auto curr_pose = RobotGetPose();
        bool x_close = units::abs(curr_pose.x) >= 27_in &&
                       units::abs(curr_pose.x) <= 30.5_in;
        bool y_close =
          units::abs(curr_pose.y) >= 43_in && units::abs(curr_pose.y) <= 51_in;
        //
        bool theta_close =
          units::abs(angleError(target_forwards_heading,
                                curr_pose.orientation)) <= 25_stDeg;

        return x_close && y_close && theta_close;
    };

    mb.moveTo(target_pose)
        .drive_vel_mp_setMaxAccel(110_inps2)
        .only_x(true, 28_in * sign_x)
        .closeThreshold(10_in)
        .timeout(2_sec)
        .reverse() |
      chain;

    chain.waitOr(exit_condition);

    // exit regardless to have better aligner
    chain.exitAll();

    pros::delay(10);

    // queue aligning motion
    mb.turnTo(target_forwards_heading)
        .turn_toleranceDuration(100_sec)
        .turn_largeToleranceDuration(100_sec)
        .constantVelocity(-10_inps) |
      async;

    // regardless of getting stuck or not we perform the same action
    intake::score_long();
    pros::delay(to_msec(score_time));

    async.exitAll();
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

    units::V2Position centerTopGoalFirst = { -9.5_in, 8.0_in };
    // units::V2Position centerBottomGoalFirst = { -10.3_in, -11_in };
    units::V2Position centerBottomGoalFirst = { -12.4_in, -13.4_in };

    alliance_t opposite_alliance =
      auto_alliance == alliance_t::red ? alliance_t::blue : alliance_t::red;

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    double angle = bl ? 90 : 270;

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(inactive);

    RobotSetPose(-48.2, 16.3 * l, angle);

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    mb.moveTo(-48, match1 * l) | run;

    // its a run here, so we can do these things
    intake::in();

    matchload(-1, l, 0.5_sec);
    score_long_goal(-1, l, 1_sec);

    matchloader::up();

    intake::in();

    mb.moveTo(-23.5, 23.4 * l).drive_vel_mp_setMaxAccel(80_inps2) | chain;

    if (bl) {
        // middle
        mb.turnTo(-11.7, 11.0).reverse() | chain;
        mb.moveTo(-11.7, 11.0).reverse() | chain;
        mb.turnTo(135)
            .constantVelocity(-5_inps)

            // infinite time motion
            .turn_toleranceDuration(100_sec)
            .turn_largeToleranceDuration(100_sec)
            .timeout(1_sec) |
          chain;

        chain.waitUntil(closeEnough({ -25.5_in, 23.4_in * l }, 10_in));
        matchloader::down();

        chain.waitUntil(closeEnough({ -11.6_in, 11.0_in }, 4_in));
        // start scoring
        // wait for balls to come up the intake
        pros::delay(300);
        // score
        intake::score_middle(1.0, 0.4);
        pros::delay(1200);
        // score for some time
    } else {
        // bottom
        mb.turnTo(-12, -12) | chain;
        mb.moveTo(-12, -12).closeThreshold(4_in).executeAfterMotion([] {
            intake::score_bottom();
        }) |
          chain;

        auto top_middle_scoring_indx = chain.getCurrentIndex();

        // align tech
        mb.turnTo(45)
            .constantVelocity(7_inps)
            .turn_toleranceDuration(100_sec)
            .turn_largeToleranceDuration(100_sec)
            .timeout(2_sec) |
          chain;

        chain.waitUntil(closeEnough({ -25.5_in, 23.4_in * l }, 10_in));
        matchloader::down();
        pros::delay(300);
        matchloader::up();
    }

    bool winging = true;
	bool fast_wing = false;

    if (winging) {
        if (bl) {
			intake::in();
            mb.moveTo(-35.9, 37.3) | chain;
            mb.turnTo(0).reverse() | chain;
            wings::down();

            mb.boomerang(-8.3, 37, 0)
                .reverse()
                .drive_maxVolt(fast_wing ? 1_volt : 0.5_volt)
                // .drive_vel_mp_maxVel(10_inps)
                // .drive_vel_maxVel(24_inps)
                // .drive_vel_accelSlew(70_inps2)
                // .drive_vel_mp_setMaxAccel(fast_wing ? 170_inps2 : 300_inps2)
                .drive_toleranceDuration(100_sec)
                .drive_largeToleranceDuration(100_sec)
                // .timeout(100_sec) |
                .timeout(100_sec) |
              chain;

            chain.wait();
        } else {
            // mb.moveTo(-35.737, -36.7) | chain;
            // mb.turnTo(0) | chain;
            // wings::down();

            // mb.boomerang(-8.1, -37.2, 0)
            //     .drive_vel_mp_maxVel(fast_wing ? 76_inps : slow_wing_speed)
            //     .drive_vel_mp_setMaxAccel(fast_wing ? 170_inps2 : 300_inps2)
            //     .drive_vel_accelSlew(70_inps2)
            //     // .drive_vel_mp_setMaxAccel(80_inps2)
            //     .drive_toleranceDuration(10.500_sec)
            //     .drive_largeToleranceDuration(100_sec)
            //     .timeout(100_sec) |
            //   chain;
            // chain.wait();
        }
    }
}

} // namespace qual_match_first
