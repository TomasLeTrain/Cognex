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
    RobotSetPose(62.4, -15.75 * l, 180);

	printf("set pose\n");

    // intake::setColorSortEnabled(true);
    intake::set(intake::intake);

	printf("before move\n");
    // pf_model.setDisabled(true);
    mb.moveTo(33, -20 * l)
        .linear_clampMaxVoltage(0.8_volt)
        .executeAfterMotion([&] {
            matchloader::set(true);
        }) |
      run;
	printf("after move\n");

    // mb.turnTo(0, 0) | run;

    // pros::delay(100);

    if (bl) {
        mb.moveTo(11.5, -12 * l).linear_clampMaxVoltage(0.5_volt) | async;
        pros::delay(500);
        matchloader::set(false);
        async.wait();
        // mb.turnTo(0, 0 * l) | run;
        intake::set(intake::scoring_middle);
    } else {
        mb.moveTo(11.8, -12.8 * l).linear_clampMaxVoltage(0.5_volt) | run;
        // pros::delay(500);
        // matchloader::set(false);
        // async.wait();
        // mb.turnTo(0, 0 * l) | run;
        intake::set(intake::scoring_bottom);
    }
    pros::delay(1200);
    intake::set(intake::intake);
    // make sure its down
    matchloader::set(false);

    // pf_model.setDisabled(false);

    mb.moveTo(40_in, -2_tile * l).reverse().linear_clampMaxVoltage(0.5_volt) |
      run;

    matchloader::set(true);

    mb.turnTo(67_in, -2_tile * l).linear_clampMaxVoltage(0.5_volt) | run;
    mb.moveTo(56.5_in, -2_tile * l).linear_clampMaxVoltage(0.8_volt) | run;

    // matchload
    pros::delay(700);

    mb.moveTo(28_in, -2_tile * l).reverse() | run;

    intake::set(intake::scoring_long);
}

} // namespace roboticon_quals
