#include "apis.h"
//
#include "auton_globals.h"
#include "autos.h"
#include "globals/blazing_globals.h"
#include "globals/vexmaps_globals.h"
#include "pros/abstract_motor.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/wings.h"


// do not do anything outside here!

namespace eagles_skills {

// you can add any variables / functions here

void run_auton() {

    RobotSetPose(-48.5, 17, 0);
    intake::setColorSortEnabled(false);

    intake::set(intake::intake);

    mb.moveTo(-46.5, 2_tile) | run;

    matchloader::set(matchloader::active);
    // pros::delay(50);

    // go into matchloader
    mb.moveTo(-58, 2_tile).drive_maxVolt(0.3_volt) | run;
    pros::delay(1000);

    // go to goal
    mb.moveTo(-24.5_in, 2_tile).reverse() | run;

    intake::set(intake::scoring_long);
    pros::delay(2000);
    intake::set(intake::intake);

    // move away from goal
    mb.moveTo(-46.394, 28.801) | run;

    intake::setColorSortEnabled(true);

    // go through the balls
    mb.moveTo(12.638, 31.201) | run;

	// color sort for red
	auto_alliance = alliance_t::red;

    intake::setColorSortEnabled(false);

    // move to right before matchloader
    mb.moveTo(48.328, 2_tile) | run;
    matchloader::set(matchloader::active);

    // go into matchloader
    mb.moveTo(58, 2_tile).drive_maxVolt(0.3_volt) | run;
    pros::delay(2000);

    // go to goal
    mb.moveTo(24.5_in, 2_tile).reverse() | run;
    intake::set(intake::scoring_long);
    pros::delay(2000);
    intake::set(intake::intake);

    // go through park
    mb.boomerang(63.27, 21.496, 270) | run;
    mb.moveTo(62.655, -26.413) | run;

    // go before matchloader
    mb.moveTo(48.328, -2_tile) | run;
    matchloader::set(matchloader::active);

    // go into matchloader
    mb.moveTo(58, -2_tile).drive_maxVolt(0.3_volt) | run;
    pros::delay(2000);

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
	mb.moveTo(-46.5,-2_tile) | run;
    matchloader::set(matchloader::active);

    // go into matchloader
    mb.moveTo(-58, -2_tile).drive_maxVolt(0.3_volt) | run;
    pros::delay(2000);

    // go to goal
    mb.moveTo(-24.5_in, -2_tile).reverse() | run;
    intake::set(intake::scoring_long);
    pros::delay(2000);
    intake::set(intake::intake);

	mb.boomerang(-62.755,-18.904,90) | run;
	mb.moveTo(-62.275,1.137) | run;
}

} // namespace eagles_skills
