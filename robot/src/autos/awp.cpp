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

void matchload(double sign_x, double sign_y, Time matchload_time) {
    auto make_matchloader_point = [](double sign_x,
                                     double sign_y) -> units::V2Position {
        Length normal_match = 46.7_in;
        return { 67.4_in * sign_x, normal_match * sign_y };
    };

    auto make_machloader_pose = [](units::V2FPosition target,
                                   Length distance) -> units::Pose {
        const auto error_unit_vector = (RobotGetPose() - target).normalize();
        auto final_point = target + distance * error_unit_vector;

        return units::Pose { final_point, final_point.angleTo(target) };
    };

    units::V2Position target_Point = make_matchloader_point(sign_x, sign_y);

    // 11 is barely achievable - 0.1 less than achievable
    Length target_dist = 10.9_in;

    auto func = [&] -> units::Pose {
        return make_machloader_pose(target_Point, target_dist);
    };

    // make sure we are matchloading
    matchloader::down();

    mb.turnTo(func())
        .turn_toleranceDuration(0_sec)
        .turn_errorTolerance(4.0_stDeg)
        .turn_velocityTolerance(400_radps) |
      run;

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
        const auto curr_pose = RobotGetPose();
        const auto error = (target_Point - curr_pose);
        const auto [forwards_error, sideways_error] =
          error.rotatedBy(-curr_pose.orientation);

        const bool close = error.magnitude() <
                           // trigger only if closes to the matchloader
                           matchload_start_distance + 5_in;

        const bool forwards_close =
          units::abs(forwards_error) < matchload_start_distance;

        // use forwards error and
        return close && forwards_close;
    };

    auto wait_result = async.waitOr(custom_exit_condition, 3_sec);

    if (wait_result == AsyncExecutorBase::motionFinished ||
        wait_result == AsyncExecutorBase::timeoutFinished) {
        // custom condition did not trigger, meaning we got stuck or
        // something else went wrong. Don't wait just exit
        async.exitAll();
    } else {
        // got to matcloader successfully, start matchloading
        async.exitAll();
        // no motions should be executing here, so setting the voltage instantly
        // should be fine

        // passive voltage forwards since motion might oscilate
        drivetrain.moveTank(0.2_volt, 0.2_volt);
        pros::delay(to_msec(matchload_time));
    }
}

void score_long_goal(double sign_x, double sign_y, Time score_time) {
    // turn to goal, reversed
    Length long_goal = 47.0_in;

    mb.moveTo(30_in, long_goal) | run;

    // auto target_backwards_heading = sign_x == -1 ? 0_stDeg : 180_stDeg;
    // auto target_forwards_heading = sign_x == -1 ? 180_stDeg : 0_stDeg;
    // auto boomerang_heading = sign_x == -1 ? 170_stDeg : 350_stDeg;

    // units::Pose target_pose = { 24_in * sign_x,
    //                             long_goal * sign_y,
    //                             target_backwards_heading };
    //
    // auto exit_condition = [&] -> bool {
    //     auto curr_pose = RobotGetPose();
    //     bool x_close =
    //       units::abs(curr_pose.x) >= 27_in && units::abs(curr_pose.x) <=
    //       40_in;
    //     // bool y_close =
    //     //   units::abs(curr_pose.y) >= 43_in && units::abs(curr_pose.y) <=
    //     //   51_in;
    //     // //
    //     // bool theta_close =
    //     //   units::abs(angleError(target_forwards_heading,
    //     //                         curr_pose.orientation)) <= 25_stDeg;
    //
    //     // return x_close && y_close && theta_close;
    //     return x_close;
    // };
    //
    // mb.moveTo(target_pose)
    //     // .drive_vel_mp_setMaxAccel(110_inps2)
    //     .only_x(true, 28_in * sign_x)
    //     .closeThreshold(7_in)
    //     // .turn_kp(13.9)
    //     .timeout(2_sec)
    //     .reverse() |
    //   chain;
    //
    // chain.waitUntil(exit_condition);
    //
    // // regardless of getting stuck or not we perform the same action
    // intake::score_long();
    //
    // pros::delay(200);
    //
    // // exit regardless to have better aligner
    // chain.exitAll();
    //
    // pros::delay(10);
    //
    // // queue aligning motion
    // mb.turnTo(target_forwards_heading)
    //     .turn_toleranceDuration(100_sec)
    //     .turn_largeToleranceDuration(100_sec)
    //     .timeout(0.3_sec)
    //     .radius(-10.5_in / 2) |
    //   async;
    // mb.turnTo(target_forwards_heading)
    //     .turn_toleranceDuration(100_sec)
    //     .turn_largeToleranceDuration(100_sec)
    //     .constantVelocity(-15_inps) |
    //   async;
    // pros::delay(to_msec(score_time));
    //
    // async.exitAll();
}

void run_auton() {
    // runs before anything else
    pre_auton();

    units::V2Position centerTopGoalFirst = { -9.5_in, 8.0_in };

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    // start auton
    RobotSetPose(-46.57, -14, 90);

    std::cout << "Stated auto: " << std::endl;

    intake::in();

    bool pushing = true;

    if (pushing) mb.moveTo(-46.57, -4.7).timeout(1.2_sec) | chain;

    mb.moveTo(-46.376, -46.1)
        .only_y(true)
        .reverse()
        // .drive_errorTolerance(2.0_in)
        // .customAngularLinearFunc([](Angle angle) {
        //     return units::cos(angle);
        // })
        .timeout(1.3_sec) |
      chain;
    chain.wait();

    matchload(-1, -1, 0.5_sec);
    score_long_goal(-1, -1, 1_sec);

    matchloader::up();

    // mb.turnTo(-1_tile, -1_tile).radius(-1.2) | chain;

    // small swing
    drivetrain.moveTank(1.0_volt, -0.8_volt);
    pros::delay(200);

    // turn towards balls
    mb.turnTo(-1_tile, -1_tile)
        .constantVelocity(-10_inps)
        .turn_errorTolerance(10.0_stDeg)
        .turn_velocityTolerance(400_radps)
        .turn_toleranceDuration(0_sec)
        .turn_chainErrorTolerance(10_stDeg) |
      chain;

    // move towards lower left balls
    mb.moveTo(-1_tile, -1_tile)
        // assume already going with some momentum
        .drive_vel_accelSlew(300_inps)
        .executeAfterMotion([] {
            intake::in();
        })
        .drive_vel_minVel(30_inps) |
      chain;

    mb.moveTo(-1_tile, 1_tile)
        .drive_vel_mp_maxVel(57_inps)
        // .drive_vel_mp_setMaxAccel(70_inps2)
        // already going fast from previous motion, slew can be faster
        .drive_vel_accelSlew(300_inps) |
      chain;

    // mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y).reverse() | chain;

    // mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
    //     .reverse()
    //     .closeThreshold(4_in)
    //   // .drive_vel_maxVel(40_inps)
    //   | chain;
    // mb.turnTo(11.574, -11.401).reverse() | chain;
    mb.moveTo(-11.7, 11.0).reverse() | chain;

    // turn towards correct heading
    mb.turnTo(135)
        .constantVelocity(-5_inps)

        // infinite time motion
        .turn_toleranceDuration(100_sec)
        .turn_largeToleranceDuration(100_sec)
        .timeout(1_sec) |
      chain;

    chain.waitUntil(closeEnough({ -1_tile, -1_tile }, 7_in));
    intake::in();
    chain.waitUntil(closeEnough({ -1_tile, 1_tile }, 14_in));
    matchloader::down();
    chain.waitUntil(closeEnough({ -11.6_in, 11.0_in }, 4_in));
    // start scoring
    // wait for balls to come up the intake
    pros::delay(200);
    // score
    intake::score_middle();
    // score for some time
    pros::delay(1000);

    // stop scoring, intake again
    intake::in();

    // exit any remaining motions
    chain.exitAll();

    // go towards matchloader
    mb.moveTo(-48_in, 46).drive_errorTolerance(2.0_in).only_y(true) | run;

    // turn to and go to matchloader

    matchload(-1, 1, 0.5_sec);
    score_long_goal(-1, 1, 1_sec);
}

} // namespace awp
