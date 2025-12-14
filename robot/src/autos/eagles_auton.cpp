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

namespace eagles_quals {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    double angle = 180;

    Length normal_match = 46.7_in;

    // printf("before set pose\n");
    RobotSetPose(62.4, -15.5 * l, angle);
    // RobotSetPose(-62.4, 15.5 * l, angle);

    intake::setColorSortEnabled(true);

    intake::set(intake::intake);

    // printf("before move\n");
    // pf_model.setDisabled(true);
    // mb.moveTo(31, -22.5 * l)
    //     .drive_maxVolt(0.7_volt)
    //     .drive_errorTolerance(5_in) |
    //   run;

    mb.moveTo(23, -23.2 * l).drive_maxVolt(0.35_volt) | async;
    async.waitUntil([&] -> bool {
        return RobotGetPose().distanceTo({ 23_in, -23.2_in * l }) < 8_in;
    });
    matchloader::down();

    async.wait();
    // printf("after move\n");

    if (bl) {
        mb.turnTo(9.7, -10 * l) | run;
        // mb.moveTo(9.7, -10 * l)
        //     .drive_maxVolt(0.5_volt)
        //     .drive_largeErrorTolerance(5_in)
        //     .timeout(2_sec) |
        //   run;
        // intake::set(intake::scoring_middle);
        // pros::delay(1000);
    } else {
        mb.turnTo(11, -10 * l) | run;
        matchloader::set(inactive);
        pros::delay(100);
        mb.moveTo(11, -10 * l).drive_maxVolt(0.5_volt) | async;
        async.waitUntil([&] -> bool {
            return RobotGetPose().distanceTo({ 11_in, -10_in * l }) < 6_in;
        });
        intake::set(intake::scoring_bottom);
        async.wait();
        pros::delay(1300);
    }

    intake::set(intake::intake);
    // make sure its down
    // matchloader::set(inactive);

    // pf_model.setDisabled(false);

    mb.moveTo(40_in, (-2_tile - 0.5_in) * l).reverse().drive_maxVolt(0.8_volt) |
      run;

    matchloader::set(active);

    mb.turnTo(67_in, RobotGetPose().y).turn_maxVolt(0.9_volt) | run;
    // mb.moveTo(59.5_in, -normal_match * l)
    mb.moveTo(59.67_in, RobotGetPose().y)
        .drive_maxVolt(0.5_volt)
        .timeout(2.5_sec)
        .k_lat(0.0) |
      run;
    // mb.distanceAtHeading(-0.5_in) | run;

    // matchload
    pros::delay(700);

    mb.turnTo(25_in, -2_tile * l).reverse() | run;
    mb.moveTo(25_in, -2_tile * l)
        .reverse()
        .timeout(1.2_sec)
        .drive_maxVolt(0.6_volt)
        .k_lat(0.0) |
      run;

    intake::set(intake::scoring_long);
}

} // namespace eagles_quals
