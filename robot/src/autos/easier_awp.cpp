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
#include "systems/wings.h"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include <iostream>

// do not do anything outside here!

namespace easier_awp {

// you can add any variables / functions here

void run_auton() {
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

    units::V2Position centerTopGoalFirst = { -8.0_in, 7.4_in };
    units::V2Position centerTopGoalSecond = { 1_in, 0_in };
    units::V2Position centerBottomGoalFirst = { 12_in, 12_in };
    units::V2Position centerBottomGoalSecond = { -12_in, -11_in };

    intake::setSkillsMiddleScoring(true);
    intake::setColorSortEnabled(false);

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

        Length slow_dist = 24_in;
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

        async.waitUntil([] -> bool {
            return units::abs(model_manager.getLocalVelocityVector().x) <
                   1_inps;
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
        Length normal_match = 46.7_in;

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

    // printf("before set pose\n");
    RobotSetPose(-46.57, 14, 270);

    intake::setColorSortEnabled(false);

    intake::in();

    mb.moveTo(-46.57, 5) | chain;

    mb.moveTo(-46.376, normal_match).reverse() | run;

    mb.turnTo(make_matchloader_point(-1, 1)).executeBeforeMotion([] {
        matchloader::down();
    }) |
      chain;
    chain.wait();

    matchload(-1, 1, 0.6_sec);

    score_long_goal(-1, 1, 0.9_sec, true);

    matchloader::up();

    mb.moveTo(-1_tile, 1_tile)
        .executeAfterMotion([] {
            // intake::set(intake::intake_bottom_balls);
            intake::set(intake::intake_bottom_top_backwards);
        })
        .drive_minVolt(0.5_volt)
        .setChainTime(0_sec) |
      chain;

    mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | chain;

    mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
        .k_lat(0.3)
        .drive_maxVolt(0.5_volt)
        .drive_kp(linear_pid.get_kp() * 0.7)
        .executeBeforeMotion([] {
            pros::Task([] {
                matchloader::down();
                pros::delay(220);
                matchloader::up();
            });
        }) |
      chain;

    chain.waitUntil(closeEnough({ -1_tile, 1_tile }, 7_in));
    matchloader::down();
    chain.waitUntil(closeEnough({ -8_in, 8_in }, 5.5_in));
    intake::set(intake::intake_disabled_open_middle);
    pros::delay(100);
    intake::set(intake::scoring_middle_bottom_balls);
    chain.exitAll();
    drivetrain.moveTank(0.1_volt, 0.1_volt);

    pros::delay(800);

    drivetrain.moveTank(-1_volt, -1_volt);
    pros::delay(200);

    // mb.moveTo(-1_tile - 2_in, 0).drive_maxVolt(0.5_volt) | chain;

    // matchloader::up();
    mb.moveTo(-1_tile + 1.5_in, -1_tile).executeBeforeMotion([] {
        pros::Task([] {
            pros::delay(200);
            intake::in();
        });
    }) |
      chain;
    chain.waitUntil(closeEnough({ -1_tile + 1.5_in, -1_tile }, 11.75_in));
    matchloader::down();

    // turn to and go to matchloader
    mb.moveTo(-48_in, -normal_match) | chain;

    mb.turnTo(make_matchloader_point(-1, -1)).executeBeforeMotion([] {
        matchloader::down();
    }) |
      chain;
    chain.wait();

    matchload(-1, -1, 0.5_sec);

    score_long_goal(-1, -1, 5_sec, true);
    drivetrain.moveTank(0_volt, 0_volt);
}

} // namespace easier_awp
