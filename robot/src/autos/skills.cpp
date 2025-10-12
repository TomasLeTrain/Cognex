/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "autos.h"

//

#include "systems/intake.h"
#include "systems/matchloader.h"

// do not do anything outside here!

namespace skills {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here
    // changePoseGetter(&smoother_model);
    // pros::delay(30);

    RobotSetPose(-63, -16, 0);
    pros::delay(50);

    intake::set(intake::intake);

    // clear park
    mb.moveTo(-63.3, 15.5).angular_clampMinVoltage(1_volt) | run;
    pros::delay(2000);

    mb.turnTo(-42, 46) | run;
    mb.moveTo(-42, 46) | run;
    pros::delay(500);

    // matchloading!
    mb.turnTo(-59, 46.5) | run;
    matchloader::set(true);
    mb.moveTo(-59, 46.5).linear_clampMaxVoltage(0.8_volt) | run;
    pros::delay(2500);
    matchloader::set(false);

    mb.moveTo(-47, 47).reverse() | run;

    mb.turnTo(-33.5, 59) | run;
    mb.moveTo(-33.5, 59) | run;

    mb.turnTo(33.5, 59) | run;
    mb.moveTo(30, 59) | run;

    pros::delay(1200);

    mb.moveTo(43, 47) | run;

    // matchloading!
    mb.turnTo(59, 46.5) | run;
    matchloader::set(true);
    mb.moveTo(59, 46.5).linear_clampMaxVoltage(0.8_volt) | run;
    pros::delay(2500);
    matchloader::set(false);

    mb.moveTo(44, 47).reverse() | run;

    // turn away from goal and turn without touching it

    // chassis.turnToHeading(130, 1000, {
    //         .direction=lemlib::AngularDirection::CW_CLOCKWISE,
    //         .minSpeed=20,
    //         .earlyExitRange=10,
    //         });
    //
    // // move towards blue park
    // chassis.moveToPose(61.3,17.5,180
    // ,1500,{.lead=0.3,.minSpeed=70,.earlyExitRange=13});
    //
    // // clear blue park
    // mb.moveTo(62, 15.5, 3000, {.minSpeed=85,.earlyExitRange=5});
    // pros::delay(2000);

    mb.turnTo(44, -47) | run;
    mb.moveTo(40, -47) | run;

    pros::delay(1200);

    // matchloading!
    mb.turnTo(47, -47) | run;
    mb.moveTo(47, -47) | run;
    mb.turnTo(59, -46.5) | run;
    matchloader::set(true);
    mb.moveTo(58, -46.5).linear_clampMaxVoltage(0.9_volt) | run;
    pros::delay(2500);
    matchloader::set(false);

    // go back and go to other corner
    mb.moveTo(47, -47).reverse() | run;

    mb.turnTo(33.5, -59) | run;
    mb.moveTo(33.5, -59) | run;

    mb.turnTo(-33.5, -59) | run;
    mb.moveTo(-30, -59) | run;

    pros::delay(2000);

    mb.turnTo(-43, -47) | run;
    mb.moveTo(-43, -47) | run;

    pros::delay(2000);

    // matchloading!
    mb.turnTo(-59, -46.5) | run;
    matchloader::set(true);
    mb.moveTo(-59, -46.5).linear_clampMaxVoltage(0.8_volt) | run;
    pros::delay(2500);
    matchloader::set(false);

    mb.moveTo(-50, -47).reverse() | run;

    mb.turnTo(-70, 0) | run;
    mb.moveTo(-70, 0).angular_clampMinVoltage(0.7_volt) | run;
}

} // namespace skills
