#include "apis.h"
//
#include "auton_globals.h"
#include "autos.h"
#include "globals/blazing_globals.h"
#include "globals/vexmaps_globals.h"
#include "pros/abstract_motor.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"

// do not do anything outside here!

namespace eagles_skills {

// you can add any variables / functions here

void run_auton() {
    RobotSetPose(-45.8, 17.14, 90);
    intake::setColorSortEnabled(false);

    intake::set(intake::intake);

    mb.moveTo(-46.5, 2_tile) | run;
    // return;

    // pros::delay(50);

    // go into matchloader
    matchloader::set(active);
    mb.turnTo(-57, 2_tile) | run;
    LaserResets({ &right_laser_model });
    mb.moveTo(-57, 2_tile).drive_maxVolt(0.4_volt).timeout(0.8_sec) | run;
    LaserResets({ &right_laser_model });

    pros::delay(2000);
    matchloader::set(inactive);

    // go to goal
    mb.moveTo(-24_in, 2_tile).reverse() | async;

    units::V2Position target;
    Length close_enough = 2_in;
    target = { -24_in, 2_tile };

    // waits until its close enough or motion finishes
    async.waitUntil([&] {
        return tracker.getPosition().distanceTo(target) < close_enough;
    });

    intake::set(intake::scoring_long);
    pros::delay(2000);
    intake::set(intake::intake);

    // wait for motion to stop if it hasn't already
    async.wait();

    // move away from goal
    mb.moveTo(-46.394, 28.801) | run;

    intake::setColorSortEnabled(true);

    // color sort for red
    auto_alliance = alliance_t::red;

    // go through the balls
    mb.moveTo(24, 31.201) | run;

    intake::setColorSortEnabled(false);

    // move to right before matchloader
    mb.moveTo(47, 2_tile) | run;

    // go into matchloader
    matchloader::set(active);
    mb.turnTo(58, 2_tile) | run;
    LaserResets({ &left_laser_model });
    mb.moveTo(58, 2_tile).drive_maxVolt(0.4_volt).timeout(0.8_sec) | run;
    LaserResets({ &left_laser_model });
    pros::delay(2000);
    matchloader::set(inactive);

    // go to goal
    mb.moveTo(23.8_in, 2_tile).reverse() | run;
    LaserResets({ &front_laser_model, &left_laser_model });
    intake::set(intake::scoring_long);
    pros::delay(2000);
    intake::set(intake::intake);

    mb.boomerang(63.27, 21.496, 270) | run;

    // go through park
    sideways_tracker.disable();
    odom_retract::set(piston_state_t::active);

    mb.moveTo(62.655, -28.413)
        .drive_maxVolt(0.6_volt)
        // effectively no slew
        .drive_accelSlew(10_volt) |
      run;

    odom_retract::set(piston_state_t::inactive);
    pros::delay(100);
    sideways_tracker.enable();

    LaserResets({ &front_laser_model, &right_laser_model });
    pros::delay(300);

    // scuff
    mb.moveTo(48, 2_tile) | run;
    mb.turnTo(48, -2_tile) | run;
    mb.moveTo(48, -2_tile) | run;
    ///

    // go before matchloader
    mb.moveTo(48.328, -2_tile) | run;
    matchloader::set(active);

    // go into matchloader
    mb.moveTo(58, -2_tile).drive_maxVolt(0.3_volt) | run;
    pros::delay(1000);
    matchloader::set(inactive);

    // go to goal
    mb.moveTo(24.5_in, -2_tile).reverse() | run;
    intake::set(intake::scoring_long);
    pros::delay(2000);
    intake::set(intake::intake);

    // move away from goal
    mb.moveTo(40.34, -60.67) | run;
    // move to other side of field
    mb.moveTo(-23.892, -61.27) | run;

    // go to right before matchloader
    mb.moveTo(-46.5, -2_tile) | run;
    matchloader::set(active);

    // go into matchloader
    mb.moveTo(-58, -2_tile).drive_maxVolt(0.3_volt) | run;
    pros::delay(1000);
    matchloader::set(inactive);

    // go to goal
    mb.moveTo(-24.5_in, -2_tile).reverse() | run;
    intake::set(intake::scoring_long);
    pros::delay(2000);
    intake::set(intake::intake);

    mb.boomerang(-62.755, -18.904, 90) | run;
    mb.moveTo(-62.275, 1.137) | run;
}

} // namespace eagles_skills
