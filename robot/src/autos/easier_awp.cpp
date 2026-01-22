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

namespace easier_awp {

void pre_auton() {
    // set the robot state to match expectations
    // done in case driver or such is run before auto
    wings::up();
    odom_retract::lowerOdom();
    matchloader::up();

    drivetrain.setBrakeMode(pros::MotorBrake::hold);
}

void run_auton() {
    // runs before anything else
    pre_auton();

    // makes point with some specified distance from the target, facing the
    // current robot position
    auto make_machloader_point = [&](units::V2FPosition target,
                                     Length distance) -> units::V2FPosition {
        auto target_angle = target.angleTo(RobotGetPose());

        return target + distance * (RobotGetPose() - target).normalize();
        // return target + units::V2Position::fromPolar(target_angle, distance);
    };

    units::V2Position centerTopGoalFirst = { -8.2_in, 7.4_in };

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    // setSmootherAlphas(10 * smoother_config.alpha_x,
    //                   10 * smoother_config.alpha_y);
    // setSmootherAlphas(1, 1);

    // printf("before set pose\n");
    RobotSetPose(-46.57, 14, 270);
    // RobotSetPose(-62.4, 15.5 , angle);

    intake::setColorSortEnabled(false);

    intake::in();

    // drivetrain.moveTank(1_volt, 1_volt);
    //
    mb.moveTo(-46.57, 5) | chain;

    mb.moveTo(-46.376, normal_match - 1.5_in)
        // .drive_maxVolt(0.5_volt)
        .drive_kp(linear_pid.get_kp() * 0.9)
        .drive_kd(linear_pid.get_kd() * 1.2)
        .drive_toleranceDuration(10_msec)
        .reverse() |
      chain;
    chain.wait();

    // somewhat good
    // turn to and go to matchloader
    // mb.turnTo(-70, normal_match).executeBeforeMotion([] {
    //     matchloader::down();
    // }) |
    //   chain;
    //
    // auto thingy = chain.getCurrentIndex();
    //
    // mb.moveTo(-60, normal_match) | chain;

    // chain.waitUntilIndex(thingy);
    // chain.waitUntil(closeEnough({ 58_in, normal_match }, 5_in));
    // pros::delay(250);
    // chain.exitAll();
    // matchloader::up();

    // turn to and go to matchloader
    mb.turnTo(-67.4_in, normal_match)
        .executeBeforeMotion([] {
            matchloader::down();
        })
        .turn_toleranceDuration(20_msec) |
      chain;
    chain.wait();

    // auto thingy = chain.getCurrentIndex();

    auto first_match_point =
      make_machloader_point({ -67.4_in, normal_match }, 7_in);

    mb.moveTo(first_match_point.x, first_match_point.y)
        .timeout(0.8_sec)
        .drive_maxVolt(0.55_volt)
        .drive_kp(linear_pid.get_kp() * 0.9)
        .drive_kd(linear_pid.get_kd() * 0.7) |
      chain;
    auto motion_indx1 = chain.getCurrentIndex();

    // chain.waitUntilIndex(thingy);
    chain.waitUntil(closeEnough(first_match_point, 4_in));
    if (motion_indx1 == chain.getFinishedIndex()) {
        // motion timed out before this executed, means we likely won't
        // matchload?
    } else {
        pros::delay(580);
    }
    chain.exitAll();
    matchloader::up();

    mb.moveTo(-25_in, long_goal)
        .reverse()
        .drive_maxVolt(0.6_volt)
        // .timeout(1.1_sec)
        .k_lat(0.0) |
      async;

    // mb.moveTo(-25_in, long_goal)
    //     .reverse()
    //     // .timeout(1.1_sec)
    //     .k_lat(0.0) |
    //   async;

    async.waitUntil(closeEnough({ -25_in, long_goal }, 7.5_in));
    intake::score_long();
    pros::delay(400);
    async.exitAll();
    // drivetrain.moveTank(-0.7_volt, -0.7_volt);

    mb.boomerang(-23_in, long_goal, 0)
        .reverse()
        .closeThreshold(100_in)
        .timeout(100_sec)
        .drive_toleranceDuration(100_sec)
        .drive_largeToleranceDuration(100_sec)
        .turn_kp(turn_drive_pid.get_kp() * 2)
        .turn_kd(turn_drive_pid.get_kd() * 0.5) |
      async;
    pros::delay(100);

    // mb.arc(0, -1.0)
    //     .reverse()
    //     .timeout(100_sec)
    //     .turn_errorTolerance(0_stDeg)
    //     .turn_largeErrorTolerance(0_stDeg)
    //     .turn_toleranceDuration(100_sec)
    //     .turn_largeToleranceDuration(100_sec)
    //     .turn_kp(angular_pid.get_kp() * 2)
    //     .turn_kd(angular_pid.get_kd() * 0.5) |
    //   async;

    pros::delay(600);
    async.exitAll();

    matchloader::up();

    // mb.turnTo(-1_tile - 3_in, -1_tile) |
    //   chain;
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
    mb.moveTo(-48_in, -normal_match)
        .drive_kp(linear_pid.get_kp() * 1.0)
        .only_y(true) |
      chain;
    // chain.wait();

    mb.turnTo(-67.4, -normal_match)
        .executeBeforeMotion([] {
            matchloader::down();
        })
        .turn_toleranceDuration(20_msec) |
      chain;
    chain.wait();

    // auto thingy = chain.getCurrentIndex();

    auto second_match_point =
      make_machloader_point({ -67.4_in, -normal_match }, 7_in);

    mb.moveTo(second_match_point.x, second_match_point.y)
        .timeout(1.0_sec)
        .drive_kp(linear_pid.get_kp() * 0.7)
        .drive_kd(linear_pid.get_kd() * 0.5) |
      chain;
    auto motion_indx2 = chain.getCurrentIndex();

    // chain.waitUntilIndex(thingy);
    chain.waitUntil(closeEnough(second_match_point, 4_in));
    if (motion_indx2 == chain.getFinishedIndex()) {
        // motion timed out before this executed, means we likely won't
        // matchload?
    } else {
        pros::delay(500);
    }
    chain.exitAll();
    matchloader::up();

    // auto thingy2 = chain.getCurrentIndex();
    // mb.moveTo(-60, -normal_match) | chain;

    // chain.waitUntilIndex(thingy2);
    // chain.waitUntil(closeEnough({ -58_in, -normal_match }, 5_in));
    // pros::delay(400);
    // chain.exitAll();
    // matchloader::up();

    mb.turnTo(-25_in, -long_goal).reverse() | chain;
    mb.moveTo(-25_in, -long_goal)
        .reverse()
        .drive_maxVolt(0.8_volt)
        // .timeout(1.1_sec)
        .k_lat(0.0) |
      chain;

    chain.waitUntil(closeEnough({ -25_in, -long_goal }, 7.5_in));
    intake::score_long();
    pros::delay(300);
    chain.exitAll();
    // drivetrain.moveTank(-0.5_volt, -0.5_volt);
    // pros::delay(200);

    // mb.arc(0, -1.0)
    //     .reverse()
    //     .timeout(100_sec)
    //     .turn_errorTolerance(0_stDeg)
    //     .turn_largeErrorTolerance(0_stDeg)
    //     .turn_toleranceDuration(100_sec)
    //     .turn_largeToleranceDuration(100_sec)
    //     .turn_kp(angular_pid.get_kp() * 2)
    //     .turn_kd(angular_pid.get_kd() * 0.75) |
    //   async;
    // pros::delay(400);
    // async.exitAll();

    mb.boomerang(-23_in, -long_goal, 0)
        .reverse()
        .closeThreshold(100_in)
        .timeout(100_sec)
        .drive_toleranceDuration(100_sec)
        .drive_largeToleranceDuration(100_sec)
        .turn_kp(turn_drive_pid.get_kp() * 2)
        .turn_kd(turn_drive_pid.get_kd() * 0.5) |
      async;

    pros::delay(200);
    matchloader::up();
    pros::delay(400);
    async.exitAll();
}

} // namespace easier_awp
