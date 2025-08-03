/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add it to "autos.h"
 */

#include "autos.h"
#include <iostream>
#include "lemlib/chassis/chassis.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"

// do not do anything outside here!

namespace skills {

// you can add any variables / functions here

void run() {
    // do whatever you want here
    // changePoseGetter(&smoother_model);
    // pros::delay(30);

    RobotSetPose(-63_in, -16_in, 0);
    pros::delay(50);

    intake::set(intake::intake);

    // clear park
    chassis.moveToPoint(-63.3, 15.5, 3000, {.minSpeed=127,.earlyExitRange=3}, false);
    pros::delay(2000);
    
    chassis.turnToPoint(-42,46,2000,{},false);
    chassis.moveToPoint(-42,46,2000,{},false);
    pros::delay(500);

    // matchloading!
    chassis.turnToPoint(-59,46.5,2000,{},false);
    matchloader::set(true);
    chassis.moveToPoint(-59,46.5,2000,{.maxSpeed=90}, false);
    pros::delay(2500);
    matchloader::set(false);

    chassis.moveToPoint(-47, 47, 2300, {.forwards=false}, false);

    chassis.turnToPoint(-33.5, 59, 2300, {}, false);
    chassis.moveToPoint(-33.5, 59, 2300, {}, false);

    chassis.turnToPoint(33.5, 59, 2300, {}, false);
    chassis.moveToPoint(30, 59, 2300, {}, false);

    pros::delay(1200);

    chassis.moveToPoint(43, 47, 2300, {}, false);

    // matchloading!
    chassis.turnToPoint(59,46.5,2000,{},false);
    matchloader::set(true);
    chassis.moveToPoint(59, 46.5, 2000, {.maxSpeed=90}, false);
    pros::delay(2500);
    matchloader::set(false);

    chassis.moveToPoint(44,47,2000,{.forwards=false},false);

    // turn away from goal and turn without touching it

    // chassis.turnToHeading(130, 1000, {
    //         .direction=lemlib::AngularDirection::CW_CLOCKWISE,
    //         .minSpeed=20,
    //         .earlyExitRange=10,
    //         },false);
    //
    // // move towards blue park
    // chassis.moveToPose(61.3,17.5,180 ,1500,{.lead=0.3,.minSpeed=70,.earlyExitRange=13},false);
    // 
    // // clear blue park
    // chassis.moveToPoint(62, 15.5, 3000, {.minSpeed=85,.earlyExitRange=5}, false);
    // pros::delay(2000);

    chassis.turnToPoint(44,-47, 2000,{},false);
    chassis.moveToPoint(40,-47, 2000,{},false);

    pros::delay(1200);

    // matchloading!
    chassis.turnToPoint(47,-47, 2000,{},false);
    chassis.moveToPoint(47,-47, 2000,{},false);
    chassis.turnToPoint(59,-46.5,2000,{},false);
    matchloader::set(true);
    chassis.moveToPoint(58,-46.5,2000,{.maxSpeed=90},false);
    pros::delay(2500);
    matchloader::set(false);

    // go back and go to other corner
    chassis.moveToPoint(47, -47, 2300, {.forwards=false}, false);

    chassis.turnToPoint(33.5, -59, 2300, {}, false);
    chassis.moveToPoint(33.5, -59, 2300, {}, false);

    chassis.turnToPoint(-33.5, -59, 2300, {}, false);
    chassis.moveToPoint(-30, -59, 2300, {}, false);

    pros::delay(2000);

    chassis.turnToPoint(-43, -47, 2300, {}, false);
    chassis.moveToPoint(-43, -47, 2300, {}, false);

    pros::delay(2000);

    // matchloading!
    chassis.turnToPoint(-59,-46.5,2000,{},false);
    matchloader::set(true);
    chassis.moveToPoint(-59,-46.5,2000,{.maxSpeed=90},false);
    pros::delay(2500);
    matchloader::set(false);

    chassis.moveToPoint(-50, -47, 2300, {.forwards = false}, false);

    chassis.turnToPoint(-70, 0, 3000, {}, false);
    chassis.moveToPoint(-70, 0, 3000, {.minSpeed=80,.earlyExitRange=8}, false);
    }

} // namespace auton1
