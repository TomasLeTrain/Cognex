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

namespace roboticon_quals {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    printf("before set pose\n");
    RobotSetPose(62.4, -15.5 * l, 180);

    intake::setColorSortEnabled(false);

    printf("set pose\n");

    // intake::setColorSortEnabled(true);
    intake::set(intake::intake);

    printf("before move\n");
    // pf_model.setDisabled(true);
    mb.moveTo(31, -22.5 * l)
        .linear_clampMaxVoltage(0.7_volt)
        .linearErrorTolerance(5_in) |
      run;

    matchloader::set(true);
    pros::delay(50);

    mb.moveTo(19, -23.2 * l).linear_clampMaxVoltage(0.3_volt) | async;
    async.wait();
    printf("after move\n");

    if (bl) {
        mb.moveTo(9.7, -10 * l)
            .linear_clampMaxVoltage(0.5_volt)
            .largeLinearErrorTolerance(5_in)
            .timeout(2_sec) |
          run;
        intake::set(intake::scoring_middle);
        pros::delay(2000);
    } else {
        mb.moveTo(10, -11 * l).linear_clampMaxVoltage(0.5_volt) | async;
        pros::delay(200);
        matchloader::set(false);
        async.wait();
        intake::set(intake::scoring_bottom);
        pros::delay(2000);
    }

    intake::set(intake::intake);
    // make sure its down
    // matchloader::set(false);

    // pf_model.setDisabled(false);

    mb.moveTo(40_in, -2_tile * l).reverse().linear_clampMaxVoltage(0.5_volt) |
      run;

    matchloader::set(true);

    mb.turnTo(67_in, -2_tile * l).linear_clampMaxVoltage(0.5_volt) | run;
    mb.moveTo(58.5_in, -2_tile * l).linear_clampMaxVoltage(0.8_volt) | run;
    mb.distanceAtHeading(-2_in) | run;

    // matchload
    pros::delay(700);

    mb.moveTo(24.5_in, -2_tile * l).reverse() | run;

    intake::set(intake::scoring_long);
}

} // namespace roboticon_quals
