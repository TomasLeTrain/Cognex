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

    // printf("before set pose\n");
    RobotSetPose(62.4, -15.5 * l, angle);
    // RobotSetPose(-62.4, 15.5 * l, angle);

    intake::setColorSortEnabled(true);

    intake::set(intake::intake);

    // printf("before move\n");
    // pf_model.setDisabled(true);
    // mb.moveTo(31, -22.5 * l)
    //     .drive_maxVolt(0.4_volt)
    //     .drive_errorTolerance(5_in) |
    //   run;
    //
    // matchloader::set(active);
    // pros::delay(50);

    mb.moveTo(19, -23.2 * l).drive_maxVolt(0.3_volt) | run;
    // printf("after move\n");

    // if (bl) {
    //     mb.moveTo(9.7, -10 * l)
    //         .drive_maxVolt(0.5_volt)
    //         .drive_largeErrorTolerance(5_in)
    //         .timeout(2_sec) |
    //       run;
    //     intake::set(intake::scoring_middle);
    //     pros::delay(2000);
    // } else {
    //     matchloader::set(inactive);
    //     pros::delay(100);
    //     mb.moveTo(10, -11 * l).drive_maxVolt(0.5_volt) | run;
    //     intake::set(intake::scoring_bottom);
    //     pros::delay(1700);
    // }

    // intake::set(intake::intake);
    // make sure its down
    // matchloader::set(inactive);

    // pf_model.setDisabled(false);

    mb.turnTo(40_in, -2_tile * l).reverse() | run;
    mb.moveTo(40_in, -2_tile * l).reverse().drive_maxVolt(0.5_volt) | run;

    // matchloader::set(active);

    mb.turnTo(67_in, -2_tile * l).turn_maxVolt(0.9_volt) | run;
    // mb.moveTo(60.5_in, -2_tile * l).drive_maxVolt(0.75_volt).timeout(2.5_sec)
    // |
    //   run;
    // mb.distanceAtHeading(-0.5_in) | run;

    // matchload
    // pros::delay(1000);

    mb.moveTo(25_in, -2_tile * l).reverse().timeout(1.2_sec) | run;

    intake::set(intake::scoring_long);
}

} // namespace eagles_quals
