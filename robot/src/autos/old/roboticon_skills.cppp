#include "apis.h"
//
#include "autos.h"
#include "globals/blazing_globals.h"
#include "globals/vexmaps_globals.h"
#include "pros/abstract_motor.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/wings.h"

// do not do anything outside here!

namespace roboticon_skills {

// you can add any variables / functions here

void run_auton() {
    RobotSetPose(-62.4, 15.5, 0);

    intake::setColorSortEnabled(false);

    intake::set(intake::intake);

    mb.moveTo(-31, 22.5).drive_maxVolt(0.7_volt).drive_errorTolerance(5_in) |
      run;

    matchloader::set(active);
    pros::delay(50);

    mb.moveTo(-19, 23.2).drive_maxVolt(0.3_volt) | run;

    mb.moveTo(-9.7, 10)
        .drive_maxVolt(0.5_volt)
        .drive_largeErrorTolerance(5_in)
        .timeout(2_sec) |
      run;
    intake::set(intake::scoring_middle);
    pros::delay(2300);

    intake::set(intake::scoring_bottom);

    mb.moveTo(-40_in, 2_tile).reverse().drive_maxVolt(0.5_volt) | run;

    matchloader::set(active);

    mb.turnTo(-67_in, 2_tile).turn_maxVolt(0.8_volt) | run;
    intake::set(intake::intake);
    // mb.moveTo(-60_in, 2_tile).drive_maxVolt(0.8_volt) | run;
    mb.moveTo(-60_in, 2_tile).drive_maxVolt(0.85_volt).timeout(2.5_sec) | run;
    // mb.distanceAtHeading(-2_in) | run;

    // matchload
    pros::delay(1000);
    intake::set(intake::intake);

    mb.moveTo(-24.5_in, 2_tile).reverse() | run;

    intake::set(intake::scoring_long);
    pros::delay(3000);
    // go back to not scoring
    // intake::set(intake::intake);
    intake::set(intake::scoring_bottom);
    matchloader::set(inactive);

    // turn around and go towards matchloader
    right_motors.set_brake_mode(pros::MotorBrake::brake);
    mb.arc(0_stDeg)
        .direction(AngularDirection::RIGHT)
        .turn_maxVolt(0.5_volt)
        .drive_maxVolt(0.5_volt) |
      run;

    drivetrain.setBrakeMode(pros::MotorBrake::coast);

    // go to other side of the field, close to the wall
    mb.moveTo(22.41, 60).drive_maxVolt(0.5_volt) | run;
    matchloader::set(active);

    // go to matchloader
    mb.boomerang(52.4_in, 2_tile, 0_stDeg).lead(0.5).drive_maxVolt(0.5_volt) |
      run;
    intake::set(intake::intake);

    mb.turnTo(67_in, 2_tile).turn_maxVolt(0.8_volt) | run;
    mb.moveTo(60_in, 2_tile).drive_maxVolt(0.85_volt).timeout(2.5_sec) | run;

    // get balls from matchloader 2
    pros::delay(1000);

    mb.moveTo(24.5_in, 2_tile).reverse() | run;

    // score
    intake::set(intake::scoring_long);
    pros::delay(3000);
    intake::set(intake::scoring_bottom);
    matchloader::set(inactive);

    // ???

    mb.distanceAtHeading(-15_in) | run;
    mb.moveTo(33, -25).drive_maxVolt(0.5_volt) | run;
    mb.moveTo(40_in, -2_tile).drive_maxVolt(0.6_volt) | run;
    intake::set(intake::intake);
    mb.turnTo(67_in, -2_tile).turn_maxVolt(0.8_volt) | run;
    mb.moveTo(60_in, -2_tile).drive_maxVolt(0.85_volt).timeout(2.5_sec) | run;
    // mb.distanceAtHeading(-2_in) | run;

    // matchload
    pros::delay(1000);

    mb.moveTo(24.5_in, -2_tile).reverse() | run;

    // score
    intake::set(intake::scoring_long);
    pros::delay(3000);
    intake::set(intake::scoring_bottom);
    matchloader::set(inactive);

    // go to other side of long goal and matchloader
    right_motors.set_brake_mode(pros::MotorBrake::brake);
    mb.arc(180_stDeg)
        .drive_maxVolt(0.5_volt)
        .direction(AngularDirection::RIGHT)
        .turn_maxVolt(0.5_volt)
        .radius(1.0) // makes it a swing
      | run;

    drivetrain.setBrakeMode(pros::MotorBrake::coast);

    // go to other side of the field, close to the wall
    mb.moveTo(-22.41, -60) | run;
    matchloader::set(active);

    // go to matchloader
    mb.boomerang(-52.4_in, -2_tile, 180_stDeg)
        .lead(0.5)
        .turn_maxVolt(0.5_volt)
        .drive_maxVolt(0.5_volt) |
      run;
    mb.turnTo(-67_in, -2_tile).turn_maxVolt(0.8_volt) | run;
    mb.moveTo(-60_in, -2_tile).drive_maxVolt(0.85_volt).timeout(2.5_sec) | run;
    // mb.distanceAtHeading(-2_in) | run;

    // get balls from matchloader 2
    pros::delay(1000);

    mb.moveTo(-24.5_in, -2_tile).reverse() | run;

    // score
    intake::set(intake::scoring_long);
}

} // namespace roboticon_skills
