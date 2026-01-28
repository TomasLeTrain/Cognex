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
#include "globals/device_globals.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"
#include "units/Angle.hpp"
#include <iostream>

// do not do anything outside here!

namespace four_ball {

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

    bool winging = true;
    bool fast_wing = false;
    Voltage slow_wing_speed = 0.5_volt;

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
        auto make_machloader_pose = [&](units::V2FPosition target,
                                        Length distance) -> units::Pose {
            // auto target_angle = target.angleTo(RobotGetPose());
            auto final_point =
              target + distance * (RobotGetPose() - target).normalize();

            return units::Pose { final_point, final_point.angleTo(target) };
        };

        Length normal_match = 46.7_in;
        units::V2Position target_Point = make_matchloader_point(sign_x, sign_y);
        Length target_dist = 7_in;

        auto func = [&] -> units::Pose {
            return make_machloader_pose(target_Point, target_dist);
        };

        // pull matchloader down regardless
        matchloader::down();

        mb.boomerang(func)
            .timeout(3_sec)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_maxVolt(0.6_volt)
            .k_lat(1.2)
            .lead(0.9) |
          async;

        Length slow_dist = 19.5_in;
        async.waitUntil([&] -> bool {
            return RobotGetPose().distanceTo(target_Point) < slow_dist;
        });

        async.exitAll();

        mb.boomerang(func)
            .timeout(3_sec)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_maxVolt(0.25_volt)
            .drive_velocityTolerance(03_inps)
            .drive_errorTolerance(10_in)
            .drive_toleranceDuration(0_sec)
            .k_lat(1.2)
            .lead(0.9)
            .executeAfterMotion([] {
                pros::delay(10);
                drivetrain.moveTank(0.25_volt, 0.25_volt);
            }) |
          async;

        Length match_timer_length_thresh = 11_in;

        // delay a bit to not include start acceleration
        pros::delay(50);

        async.waitUntil([&] -> bool {
            return (units::abs(model_manager.getLocalVelocityVector().x) <
                    1_inps) &&
                   (RobotGetPose().distanceTo(target_Point) <
                    match_timer_length_thresh);
        });
        // here the robot is close to still, start matchloading

        pros::delay(to_msec(matchload_time));

        async.exitAll();
    };

    auto score_long_goal = [](double sign_x,
                              double sign_y,
                              Time score_time,
                              bool from_matchloader = false) {
        // turn to goal, reversed
        // mb.turnTo(25_in * sign_x, long_goal * sign_y).reverse() |
        // chain; mb.moveTo(25_in * sign_x, long_goal * sign_y)
        //     .reverse()
        //     .k_lat(0.0) |
        //   chain;

        Length long_goal = 47.1_in;

        units::Pose target_pose = { 26_in * sign_x,
                                    long_goal * sign_y,
                                    sign_x == -1 ? 0_stDeg : 180_stDeg };

        units::Pose other_target_pose = { 24_in * sign_x,
                                          long_goal * sign_y,
                                          sign_x == -1 ? 0_stDeg : 180_stDeg };

        if (from_matchloader)
            mb.arc(target_pose, -1.3)
                .reverse()
                // .drive_chainErrorTolerance()
                .setChainTime(0_sec)
                .drive_minVolt(0.2_volt) |
              chain;
        else
            mb.turnTo(target_pose).reverse().setChainTime(0_sec) | chain;

        mb.boomerang(target_pose).reverse() | chain;

        Length slow_dist = 12_in;
        Length score_dist = 7.5_in;

        chain.waitUntil([&] -> bool {
            return RobotGetPose().distanceTo(target_pose) < slow_dist;
        });

        // exit current boomerang
        chain.exitAll();

        // go into new which is slower
        mb.boomerang(target_pose)
            .drive_maxVolt(0.5_volt)
            .reverse()

            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            // .drive_maxVolt(0.25_volt)
            .drive_velocityTolerance(03_inps)
            .drive_errorTolerance(6_in)
            .drive_toleranceDuration(0_sec)
            .setChainTime(0_sec)

          // .closeThreshold(100_in)
          // .timeout(5_sec)
          // .drive_toleranceDuration(100_sec)
          // .drive_largeToleranceDuration(100_sec)
          // .turn_kp(turn_drive_pid.get_kp() * 2)
          // .turn_kd(turn_drive_pid.get_kd() * 0.5)
          | chain;

        mb.boomerang(other_target_pose)
            // .drive_maxVolt(0.3_volt)
            .reverse()
            .closeThreshold(100_in)
            .timeout(5_sec)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_ki(0)
            .turn_kp(turn_drive_pid.get_kp() * 1.0)
            .turn_kd(turn_drive_pid.get_kd() * 0.8) |
          chain;

        chain.waitUntil(closeEnough(target_pose, score_dist));
        intake::score_long();
        pros::delay(to_msec(score_time));
        chain.exitAll();
        // intake::set(intake::intake_disabled);
        // drivetrain.moveTank(0_volt, 0_volt);
    };


    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    RobotSetPose(-48.2, 16.3 * l, angle);


    mb.moveTo(-48, (match1 - 0.1_in) * l) | run;

    // its a run here, so we can do these things
    intake::in();
    // turn to and go to matchloader
    matchloader::down();
    mb.turnTo(make_matchloader_point(-1, l)).turn_toleranceDuration(25_msec) |
      run;

    matchload(-1, l, 0.30_sec);
    score_long_goal(-1, l, 1_sec, true);

    matchloader::up();

    matchloader::up();
    intake::score_long();

    if (winging) {
        if (bl) {
            mb.moveTo(-35.737, 37.3) | chain;
            mb.turnTo(0).reverse() | chain;
            wings::down();
            mb.boomerang(-8.0, 37, 0)
                .reverse()
                .drive_maxVolt(fast_wing ? 1.0_volt : slow_wing_speed)
                .drive_toleranceDuration(100_sec)
                .drive_largeToleranceDuration(100_sec)
                // .timeout(100_sec) |
                .timeout(100_sec) |
              chain;

            // mb.boomerang(-9, 37, 340)
            //     .reverse()
            //     .drive_maxVolt(fast_wing ? 1.0_volt : slow_wing_speed)
            //     .drive_toleranceDuration(100_sec)
            //     .drive_largeToleranceDuration(100_sec)
            //     // .timeout(100_sec) |
            //     .timeout(100_sec) |
            //   chain;

            // mb.turnTo(340)
            //     .reverse()
            //     // .turn_maxVolt(fast_wing ? 1.0_volt : slow_wing_speed)
            //     .turn_toleranceDuration(100_sec)
            //     .turn_largeToleranceDuration(100_sec)
            //     // .timeout(100_sec) |
            //     .timeout(100_sec) |
            //   chain;

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

            mb.boomerang(-8.6, -37.2, 0)
                .drive_maxVolt(fast_wing ? 1.0_volt : slow_wing_speed)
                .drive_toleranceDuration(10.500_sec)
                .drive_largeToleranceDuration(100_sec)
                .timeout(100_sec) |
              chain;
        }
    }
}

} // namespace four_ball
