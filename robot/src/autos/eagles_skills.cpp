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
    RobotSetPose(-47.9, 14.8, 90);
    intake::setColorSortEnabled(false);

    intake::set(intake::intake);

    // complete reset
    // LaserResets({ &left_laser_model });

    Length long_goal = 47.0_in;
    Length normal_match = 46.7_in;

    Length match1 = normal_match;
    Length match2 = normal_match;
    Length match3 = -(normal_match);
    Length match4 = -(normal_match);

    mb.moveTo(-46.5, match1 - 3_in) | run;
    // return;

    // pros::delay(50);

    // go into matchloader
    // mb.turnTo(-57, match1) | run;
    mb.turnTo(-70, match1) | run;

    // should reset y
    // LaserResets({ &right_laser_model });
    matchloader::set(active);
    pros::delay(300);

    // mb.moveTo(-57, 2_tile).drive_maxVolt(0.4_volt).timeout(0.8_sec) | run;
    // mb.distanceAtHeading(10_in).drive_maxVolt(0.4_volt).timeout(0.8_sec) |
    // run;
    // mb.distanceAtHeading(13.8_in).timeout(0.9_sec) | run;
    mb.moveTo(-57.7, match1) | run;

    // LaserResets({ &right_laser_model, &front_laser_model });
    drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);
    pros::delay(2200);
    drivetrain.setBrakeMode(pros::v5::MotorBrake::coast);

    // go to goal
    // mb.moveTo(-23.9_in, 2_tile).reverse().timeout(1_sec) | run;
    mb.moveTo(-26.6_in, 46.8_in).reverse() | run;

    std::cout << tracker.getPosition().x.convert(in) << " "
              << tracker.getPosition().y.convert(in) << std::endl;

    // units::V2Position target;
    // Length close_enough = 2_in;
    // target = { -24_in, 2_tile };
    //
    // // waits until its close enough or motion finishes
    // while (!(tracker.getPosition().distanceTo(target) < close_enough)) {
    //     pros::delay(10);
    // };
    //
    intake::set(intake::scoring_long);
    pros::delay(3200);
    intake::set(intake::intake);
    matchloader::set(inactive);
    // LaserResets({ &right_laser_model });

    // move away from goal
    mb.moveTo(-46.394, 28.801) | run;

    // intake::setColorSortEnabled(true);

    // color sort for red
    // auto_alliance = alliance_t::red;
    intake::set(intake::scoring_long);

    // go through the balls
    mb.moveTo(24, 31.201) | run;

    // balls
    // LaserResets({ &front_laser_model });

    // intake::setColorSortEnabled(false);

    // move to right before matchloader
    mb.moveTo(44, match2) | run;

    // go into matchloader
    intake::set(intake::intake);
    matchloader::set(active);
    // mb.turnTo(58, match2) | run;
    mb.turnTo(70, match2) | run;

    // LaserResets({ &left_laser_model });
    // mb.moveTo(58.5, 2_tile).drive_maxVolt(0.4_volt).timeout(0.8_sec) | run;
    // mb.distanceAtHeading(14.2_in).timeout(1_sec) | run;
    mb.moveTo(57.7, match2) | run;

    // LaserResets({ &left_laser_model });

    // pros::delay(200);
    drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);
    pros::delay(2200);
    drivetrain.setBrakeMode(pros::v5::MotorBrake::coast);

    // LaserResets({ &left_laser_model });
    // pros::delay(1000);
    // matchloader::set(inactive);

    // go to goal
    mb.moveTo(26.6_in, 46.8_in).reverse().timeout(1_sec) | run;

    // LaserResets({ &left_laser_model });
    intake::set(intake::scoring_long);
    pros::delay(3200);
    intake::set(intake::intake);
    matchloader::set(inactive);

    // mb.boomerang(63.27, 21.496, 270) | run;

    // go through park
    // sideways_tracker.disable();
    // odom_retract::set(piston_state_t::active);
    //
    // mb.moveTo(62.655, -28.413)
    //     .drive_maxVolt(0.6_volt)
    //     // effectively no slew
    //     .drive_accelSlew(10_volt) |
    //   run;
    //
    // odom_retract::set(piston_state_t::inactive);
    // pros::delay(100);
    // sideways_tracker.enable();
    //
    // LaserResets({ &front_laser_model, &right_laser_model });
    // pros::delay(300);

    // scuff
    intake::set(intake::scoring_long);

    mb.moveTo(41, 2_tile) | run;
    mb.turnTo(44, -2_tile) | run;
    // mb.moveTo(41, -1_tile).drive_maxVolt(0.8_volt) | run;
    // reset all
    // pros::delay(400);
    // LaserResets({ &front_laser_model, &left_laser_model });
    // pros::delay(50);
    ///

    // go before matchloader
    mb.moveTo(44, match3 + 4_in) | run;
    // LaserResets({ &front_laser_model });

    matchloader::set(active);
    intake::set(intake::intake);

    // go into matchloader
    // mb.turnTo(58, match3) | run;
    mb.turnTo(70, match3) | run;

    // mb.distanceAtHeading(14.7_in).timeout(1_sec) | run;
    mb.moveTo(57.7, match3) | run;

    pros::delay(100);
    drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);
    // LaserResets({ &right_laser_model });
    pros::delay(2200);
    drivetrain.setBrakeMode(pros::v5::MotorBrake::coast);

    // go to goal
    // LaserResets({&right_laser_model});

    mb.moveTo(26.6_in, -46.8_in).reverse() | run;
    intake::set(intake::scoring_long);
    pros::delay(3200);
    intake::set(intake::intake);
    matchloader::set(inactive);

    // move away from goal
    mb.moveTo(40.34, -60.67) | run;
    // move to other side of field
    mb.moveTo(-23.892, -61.27) | run;
    // pros::delay(100);
    // LaserResets({ &front_laser_model, &left_laser_model });
    // pros::delay(50);

    // go to right before matchloader
    mb.moveTo(-46.5, match4) | run;
    matchloader::set(active);

    // go into matchloader
    // mb.turnTo(-58, match4) | run;
    mb.turnTo(-70, match4) | run;
    // LaserResets({ &left_laser_model });

    // mb.distanceAtHeading(13.5_in).timeout(1_sec) | run;
    mb.moveTo(-57.7, match4) | run;

    drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);
    pros::delay(2200);
    drivetrain.setBrakeMode(pros::v5::MotorBrake::coast);

    // go to goal
    mb.moveTo(-26.6_in, -46.8_in).reverse() | run;
    intake::set(intake::scoring_long);
    pros::delay(3200);
    intake::set(intake::intake);
    matchloader::set(inactive);

    // mb.moveTo(-40_in, -2_tile) | run;
    // mb.turnTo(180) | run;

    mb.boomerang(-58.755, -18.904, 90)
        // .reverse()
        // .lead(0.35)
        .lead(0.4)
        .drive_maxVolt(0.5_volt) |
      run;

    odom_retract::set(active);
    sideways_tracker.disable();

    // mb.moveTo(-62.275, 1.137) | run;
    mb.moveTo(-62.275, 1.137)
        // go max?
        .drive_accelSlew(100_volt)
        .drive_minVolt(0.3_volt)
        .drive_ErrorTolerance(4_in)
        .drive_ToleranceDuration(0.01_sec) |
      run;

    // mb.moveTo(-62.275, 1.137).reverse().drive_backwardsAccelSlew(100_volt) |
    // run;

    intake::set(intake::intake_disabled);
}

} // namespace eagles_skills
