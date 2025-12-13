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

namespace neocity_awp {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    double angle = 90;

    // printf("before set pose\n");
    RobotSetPose(-48.3,
                 16.5 * l,
                 bl ? angle :
                      units::constrainAngle2pi(-angle * deg).convert(deg));
    // RobotSetPose(-62.4, 15.5 * l, angle);

    intake::setColorSortEnabled(true);

    intake::in();

    matchloader::down();

    mb.moveTo(-44, normal_match * l) | run;
    // turn to and go to matchloader
    mb.turnTo(-70, normal_match * l) | run;
    mb.moveTo(-60, normal_match * l) | run;

    pros::delay(2000);
    matchloader::up();

    mb.moveTo(-25_in, long_goal * l).reverse().timeout(1.1_sec).k_lat(0.0) |
      run;
    async.waitUntil([&] -> bool {
        return RobotGetPose().distanceTo({ -25_in, long_goal * l }) < 4_in;
    });
    intake::score_long();
    async.wait();
    pros::delay(2000);

    matchloader::up();
    intake::in();

    mb.turnTo(-1_tile, 1_tile * l) | run;
    mb.moveTo(-1_tile, 1_tile * l) | async;
    async.waitUntil([&] -> bool {
        return RobotGetPose().distanceTo({ -1_tile, 1_tile * l }) < 6_in;
    });
    matchloader::down();
    async.wait();

    matchloader::up();
    mb.moveTo(-1_tile, -1_tile * l) | async;
    async.waitUntil([&] -> bool {
        return RobotGetPose().distanceTo({ -1_tile, -1_tile * l }) < 6_in;
    });
    matchloader::down();
    async.wait();

    mb.turnTo(13_in, 12.8_in * l) | run;
    matchloader::up();
    mb.moveTo(13_in, 12.8_in * l) | run;
    intake::score_bottom();
    pros::delay(1000);

    mb.moveTo(-44, -normal_match * l).reverse() | run;
    matchloader::down();
    mb.turnTo(-70, -normal_match * l) | run;
    // mat
    // go to matchloader
    mb.moveTo(-60, -normal_match * l) | run;
    pros::delay(1000);

    // go to final goal
    mb.moveTo(-25_in, -long_goal * l).reverse().timeout(1.1_sec).k_lat(0.0) |
      run;
    async.waitUntil([&] -> bool {
        return RobotGetPose().distanceTo({ -25_in, -long_goal * l }) < 4_in;
    });
    intake::score_long();
    async.wait();
    // pros::delay(2000);
}

} // namespace neocity_awp
