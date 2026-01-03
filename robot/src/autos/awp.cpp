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
#include "systems/wings.h"
#include "units/Angle.hpp"
#include <iostream>

// do not do anything outside here!

namespace awp {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    units::V2Position centerTopGoalFirst = { -8.0_in, 7.4_in };

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    // printf("before set pose\n");
    RobotSetPose(-46.57, -1.5, 90);
    // RobotSetPose(-62.4, 15.5 , angle);

    // intake::setColorSortEnabled(true);

    intake::in();

    mb.moveTo(-46.376, 7) | chain;

    mb.moveTo(-46, -normal_match)
        .drive_kp(linear_pid.get_kp() * 0.8)
        .reverse() |
      chain;
    chain.wait();

    // turn to and go to matchloader
    mb.turnTo(-70, -normal_match).executeBeforeMotion([] {
        matchloader::down();
    }) |
      chain;
    auto thingy = chain.getCurrentIndex();
    mb.moveTo(-59, -normal_match) | chain;
    chain.waitUntilIndex(thingy);
    chain.waitUntil(closeEnough({ -58_in, -normal_match }, 5_in));
    pros::delay(600);
    chain.exitAll();
    // matchloader::up();

    mb.moveTo(-25_in, -long_goal)
        .reverse()
        // .timeout(1.1_sec)
        .k_lat(0.0) |
      async;
    async.waitUntil(closeEnough({ -25_in, -long_goal }, 7_in));
    intake::score_long();
    pros::delay(1300);
    async.exitAll();

    matchloader::up();

    // mb.turnTo(-1_tile - 3_in, -1_tile) |
    //   chain;
    mb.moveTo(-1_tile - 2_in, -1_tile).executeAfterMotion([] {
        pros::delay(400);
        intake::in();
    }) |
      chain;
    mb.moveTo(-1_tile - 2_in, 0).drive_maxVolt(0.5_volt) | chain;

    // matchloader::up();
    mb.moveTo(-1_tile, 1_tile) | chain;

    chain.waitUntil(closeEnough({ -1_tile - 2_in, -1_tile }, 10_in));
    //    matchloader::down();
    // pros::d
    // chain.wait();

    chain.waitUntil(closeEnough({ -1_tile, 1_tile }, 10_in));
    matchloader::down();
    chain.wait();

    // if (bl) {
    //     mb.turnTo(-11_in, -10_in) | run;
    //     matchloader::up();
    //     mb.moveTo(-11_in, -10_in) | async;
    //     async.waitUntil(closeEnough({ -11_in, -10_in }, 6_in));
    //     intake::score_bottom();
    //     pros::delay(1200);
    //     async.wait();
    // } else {
    //     mb.turnTo(-11_in, -10_in) | run;
    //     matchloader::up();
    //     mb.moveTo(-11_in, -10_in) | async;
    //     async.waitUntil(closeEnough({ -11_in, -10_in }, 6_in));
    //     intake::score_middle();
    //     pros::delay(1200);
    //     async.wait();
    // }

    // mb.moveTo(-44, normal_match) | run;
    // matchloader::down();
    // mb.turnTo(-70, normal_match) | run;
    //
    // // go to matchloader
    // mb.moveTo(-60, normal_match) | run;
    // pros::delay(1000);
    mb.moveTo(-47_in, long_goal).drive_maxVolt(0.5_volt) | async;
    mb.turnTo(-25_in, long_goal) | async;

    // go to final goal
    mb.moveTo(-25_in, long_goal).reverse() | async;
    async.waitUntil(closeEnough({ -25_in, long_goal }, 5_in));
    intake::score_long();
    pros::delay(1300);
    async.exitAll();

    intake::set(intake::intake_bottom_balls);

    mb.moveTo(-58, normal_match) | run;
    pros::delay(200);
    mb.moveTo(-23.6, 23.6).reverse() | run;
    mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | run;
    mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
        .drive_maxVolt(0.5_volt) |
      run;
    intake::out();
    pros::delay(150);
    intake::set(intake::scoring_middle_bottom_balls);
}

} // namespace awp
