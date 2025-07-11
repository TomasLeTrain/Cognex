/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add it to "autos.h"
 */

#include "autos.h"
#include <iostream>
#include "globals.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/abstract_motor.hpp"
#include "systems/intake.h"
#include "motions/arc.h"

// do not do anything outside here!

namespace skills2 {

// you can add any variables / functions here

void run() {
    // do whatever you want here

    pros::Task smoother_task {[&] {
    while (true) {
        int start_time = pros::millis();
        printf(
          "start generation\nstart distances\nend distances\nstart " "parti" "cles" "\n");

        printf("%.1f %.1f %.1f\n",
               pf_motion_model.getPose().x.convert(in),
               pf_motion_model.getPose().y.convert(in),
               0.0);
        printf("%.1f %.1f %.1f\n",
               pf_model.getPose().x.convert(in),
               pf_model.getPose().y.convert(in),
               5.0);
        printf("%.1f %.1f %.1f\n",
               smoother_model.getPose().x.convert(in),
               smoother_model.getPose().y.convert(in),
               10.0);

        printf(
          "end particles\ntotal weight: 0, time taken: 30000, " "timestamp:" " %d\n",
          start_time);
        printf("things done:1,1,0,%d\n",16384);
        printf("prediction:%.1f,%.1f,%.1f\n",
               smoother_model.getPose().x.convert(in),
               smoother_model.getPose().y.convert(in),
               smoother_model.getPose().orientation.convert(deg));
        printf("end generation\n");

        pros::delay(20);
    }
    }};

    pf_model.setPose({-63.3_in, -17.5_in, 90_stDeg});
    RobotSetPose(-63.3_in, -17.5_in, 0);

    pros::delay(12);

    // clear park
    // chassis.moveToPoint(-63.3, 17.5, 3000, {}, false);

    // pros::delay(1000);
    left_motor_group.move(0);
    right_motor_group.move(0);

    // chassis.turnToHeading(24,2000,{.minSpeed=20,.earlyExitRange=2},false);

    // motions::moveArc(20.5_in, 140, 28_in / sec, lemlib::AngularDirection::CW_CLOCKWISE, 5000, {},false);
    motions::moveArc(13.5_in, 130, 22_in / sec, lemlib::AngularDirection::CW_CLOCKWISE, 2000, {.maxAccel=10_rpm}, false);
    // left_motor_group.set_brake_mode_all(pros::MotorBrake::hold);
    // right_motor_group.set_brake_mode_all(pros::MotorBrake::hold);

    // move towards first corner
    // chassis.moveToPose(-29,31,145,2000,{.lead=0.5},false);

    // move towards center goal to score
    chassis.moveToPoint(-14.3, 14, 2000, {.maxSpeed=40}, false);

    // score on center top goal
    pros::delay(3000);

    // swing to face second corner
    chassis.swingToPoint(-23.3,-22.5,lemlib::DriveSide::LEFT, 2000, {
            .direction=lemlib::AngularDirection::CW_CLOCKWISE
            },false);

    // move to second corner
    chassis.moveToPoint(-25,-23, 2000, {}, false);

    // turn to and go the third corner
    chassis.turnToPoint(23.5,-23.5, 2000, {}, false);
    chassis.moveToPoint(23.5,-23.5, 2000, {}, false);

    // go to matchloader
    chassis.moveToPose(56.5,-47.5,90,2000,{ .lead=0.5 },false);
    pros::delay(3000);

    // back up to go to goal
    chassis.moveToPoint(50, -47.5, 2000, { .forwards=false }, false);

    // turn to and score on long goal
    chassis.turnToPoint(0, -47.5, 2000, {}, false);
    chassis.moveToPoint(32, -46.5, 2000, {}, false);
    pros::delay(5500);

    // turn away from goal and turn without touching it
    chassis.swingToHeading(90,lemlib::DriveSide::RIGHT, 800, {
            .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
            .minSpeed=20,
            .earlyExitRange=10,
            },false);
    // chassis.turnToHeading(43, 1000, {
    //         .minSpeed=10,
    //         .earlyExitRange=10,
    //         },false);

    // move towards blue park
    chassis.moveToPose(62.3,-17.5,0 ,2000,{.lead=0.9},false);

    // clear park
    chassis.moveToPoint(63, 17.5, 2000, {}, false);

    // get to matchloader
    chassis.moveToPose(57.3,46.7,0 ,2000,{.lead=0.5},false);
    pros::delay(3000);

    chassis.moveToPoint(46.8, 46.7, 2000, {.forwards=false}, false);

    // turn to and go to fourth corner
    chassis.turnToPoint(32.3, 31.5, 2000, {}, false);
    chassis.moveToPoint(32.3, 31.5, 2000, {}, false);

    // score on center goal
    chassis.moveToPoint(13.7, 13.6, 2000, {}, false);
    pros::delay(3000);

    // back up and go to third matchloader
    chassis.moveToPoint(18, 19.5, 2000, {}, false);
    chassis.turnToPoint(-54, 50, 2000, {}, false);

    // move to matchloader
    chassis.moveToPoint(-52.6, 46.7, 2000, {}, false);
    chassis.turnToPoint(-71, 46.7, 2000, {}, false);
    chassis.moveToPoint(-55, 46.7, 2000, {}, false);
    pros::delay(3000);

    // back up, turn to and go to long goal
    chassis.moveToPoint(-50,46.7,2000,{},false);
    chassis.turnToPoint(0,46.7,2000,{},false);
    chassis.moveToPoint(-32, 46.7,2000,{},false);

    // swing and run for the park
    chassis.swingToPoint(-63,0,lemlib::DriveSide::RIGHT, 800, {
            .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
            .minSpeed=25,
            .earlyExitRange=10
            },false);
    chassis.moveToPoint(-63,0,2000,{},false);
    }

} // namespace auton1
