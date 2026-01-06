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
#include <iostream>

// do not do anything outside here!

namespace easier_awp {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    units::V2Position centerTopGoalFirst = { -8.2_in, 7.4_in };

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    // setSmootherAlphas(10 * smoother_config.alpha_x,
    //                   10 * smoother_config.alpha_y);
    setSmootherAlphas(1, 1);

    // printf("before set pose\n");
    RobotSetPose(-46.57, 14, 270);
    // RobotSetPose(-62.4, 15.5 , angle);

    intake::setColorSortEnabled(false);

    intake::in();

    // drivetrain.moveTank(1_volt, 1_volt);
    mb.moveTo(-46.57, 5) | chain;
    pros::delay(300);
    // mb.moveTo(-46.376, -5) | chain;

    mb.moveTo(-46.376, normal_match)
        // .drive_maxVolt(0.5_volt)
        .drive_kp(linear_pid.get_kp() * 0.8)
        .drive_toleranceDuration(50_msec)
        .reverse() |
      chain;
    chain.wait();

    // turn to and go to matchloader
    mb.turnTo(-70, normal_match).executeBeforeMotion([] {
        matchloader::down();
    }) |
      chain;

    auto thingy = chain.getCurrentIndex();
    mb.moveTo(-60, normal_match) | chain;

    chain.waitUntilIndex(thingy);
    chain.waitUntil(closeEnough({ 58_in, normal_match }, 5_in));
    pros::delay(250);
    chain.exitAll();
    // matchloader::up();

    mb.moveTo(-25_in, long_goal)
        .reverse()
        // .timeout(1.1_sec)
        .k_lat(0.0) |
      async;

    async.waitUntil(closeEnough({ -25_in, long_goal }, 7.5_in));
    intake::score_long();
    pros::delay(300);
    async.exitAll();
    drivetrain.moveTank(-0.7_volt, -0.7_volt);
    pros::delay(200);

    mb.arc(0, -1.0)
        .reverse()
        .timeout(100_sec)
        .turn_errorTolerance(0_stDeg)
        .turn_largeErrorTolerance(0_stDeg)
        .turn_toleranceDuration(100_sec)
        .turn_largeToleranceDuration(100_sec)
        .turn_kp(angular_pid.get_kp() * 2)
        .turn_kd(angular_pid.get_kd() * 0.5) |
      async;
    pros::delay(400);
    async.exitAll();

    matchloader::up();

    // mb.turnTo(-1_tile - 3_in, -1_tile) |
    //   chain;
    mb.moveTo(-1_tile, 1_tile)
        .executeAfterMotion([] {
            pros::delay(400);
            intake::set(intake::intake_bottom_balls);
        })
        .drive_minVolt(0.5_volt)
        .setChainTime(0_sec) |
      chain;

    mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | chain;

    mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
        .k_lat(0.3)
        .drive_maxVolt(0.3_volt)
        .executeBeforeMotion([] {
            pros::Task([] {
                matchloader::down();
                pros::delay(250);
                matchloader::up();
            });
        }) |
      chain;

    chain.waitUntil(closeEnough({ -1_tile, 1_tile }, 7_in));
    matchloader::down();
    chain.waitUntil(closeEnough({ -8_in, 8_in }, 5.5_in));
    intake::out();
    pros::delay(150);
    intake::set(intake::scoring_middle_bottom_balls);
    chain.exitAll();
    drivetrain.moveTank(0.1_volt, 0.1_volt);

    pros::delay(800);

    drivetrain.moveTank(-1_volt, -1_volt);
    pros::delay(200);

    // mb.moveTo(-1_tile - 2_in, 0).drive_maxVolt(0.5_volt) | chain;

    // matchloader::up();
    mb.moveTo(-1_tile, -1_tile + 1_in).executeBeforeMotion([] {
        pros::Task([] {
            pros::delay(200);
            intake::in();
        });
    }) |
      async;
    async.waitUntil(closeEnough({ -1_tile, -1_tile }, 7_in));
    matchloader::down();
    async.wait();

    // mb.moveTo(-47_in, -long_goal) | chain;

    // mb.turnTo(0) | chain;
    // auto thing = chain.getCurrentIndex();

    // go to final goal
    // mb.moveTo(-25_in, -long_goal).reverse() | chain;
    // mb.arc(0, -1.0)
    //     .timeout(100_sec)
    //     .turn_errorTolerance(0_stDeg)
    //     .turn_largeErrorTolerance(0_stDeg)
    //     .turn_toleranceDuration(100_sec)
    //     .turn_largeToleranceDuration(100_sec) |
    //   chain;
    //
    // chain.waitUntil(closeEnough({ -1_tile - 2_in, -1_tile }, 10_in));
    // matchloader::down();
    //
    // chain.waitUntilIndex(thing);
    // chain.waitUntil(closeEnough({ -24_in, -long_goal }, 7.5_in));
    // intake::score_long();
    // pros::delay(300);
    // async.exitAll();
    // drivetrain.moveTank(-0.7_volt, -0.7_volt);
    // pros::delay(200);
    //
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
    // pros::delay(400);
    // async.exitAll();

    // mb.moveTo(46.376, -normal_match + 2.5_in)
    //     // .drive_maxVolt(0.5_volt)
    //     .drive_kp(linear_pid.get_kp() * 1.0)
    //     .reverse() |
    //   chain;
    // chain.wait();

    // turn to and go to matchloader
    mb.moveTo(-45_in, -normal_match)
        .drive_kp(linear_pid.get_kp() * 0.86)
        .only_y(true) |
      run;
    // chain.wait();

    mb.turnTo(-70, -normal_match).executeBeforeMotion([] {
        matchloader::down();
    }) |
      chain;

    auto thingy2 = chain.getCurrentIndex();
    mb.moveTo(-60, -normal_match) | chain;

    chain.waitUntilIndex(thingy2);
    chain.waitUntil(closeEnough({ -58_in, -normal_match }, 5_in));
    pros::delay(500);
    chain.exitAll();
    // matchloader::up();

    mb.turnTo(-25_in, -long_goal).reverse() | chain;
    mb.moveTo(-25_in, -long_goal)
        .reverse()
        // .timeout(1.1_sec)
        .k_lat(0.0) |
      chain;

    chain.waitUntil(closeEnough({ -25_in, -long_goal }, 7.5_in));
    intake::score_long();
    pros::delay(300);
    chain.exitAll();
    drivetrain.moveTank(-0.7_volt, -0.7_volt);
    pros::delay(200);

    mb.arc(0, -1.0)
        .reverse()
        .timeout(100_sec)
        .turn_errorTolerance(0_stDeg)
        .turn_largeErrorTolerance(0_stDeg)
        .turn_toleranceDuration(100_sec)
        .turn_largeToleranceDuration(100_sec)
        .turn_kp(angular_pid.get_kp() * 2)
        .turn_kd(angular_pid.get_kd() * 0.5) |
      async;
    pros::delay(400);
    async.exitAll();

    matchloader::up();
}

} // namespace easier_awp
