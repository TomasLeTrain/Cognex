/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add it to "autos.h"
 */

#include "autos.h"
#include <iostream>
#include "lemlib/chassis/chassis.hpp"
#include "systems/intake.h"

// do not do anything outside here!

namespace skills {

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

    RobotSetPose(-63.3_in, -17.5_in, 0);

    // std::cout << RobotGetPose().x << " " << RobotGetPose().y << " " << RobotGetPose().orientation << std::endl;
    // printf("chassis: %f %f %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

    // chassis.turnToHeading(0,2000,{},false);
    // chassis.turnToHeading(0,9000,{},false);
    chassis.moveToPoint(-63.3, 17.5, 3000, {}, false);

    // chassis.moveToPose(-55.5, 46.5, 270, 3000, {}, false);
    // chassis.moveToPoint(-48, 42, 2000, {}, false);
    // chassis.turnToPoint(-70, 46.5, 2000, {}, false);
    // chassis.moveToPoint(-55.5, 46.5, 2000, {}, false);

    chassis.moveToPose(-55.5,46.5,270,3000,{.lead=0.5},false);

    // pros::delay(3000);

    chassis.swingToHeading(150,lemlib::DriveSide::RIGHT, 2000, {
            .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
            .minSpeed=25,
            .earlyExitRange=10
            }, false);

    chassis.moveToPoint(-22.2, 22.2, 2000, {}, false);

    chassis.turnToPoint(0,28, 2000, {}, false);
    chassis.moveToPoint(0,28, 2000, {}, false);

    chassis.turnToPoint(0,34, 2000, {}, false);

    chassis.swingToHeading(70,lemlib::DriveSide::LEFT, 2000, {
            .direction=lemlib::AngularDirection::CW_CLOCKWISE }, false);

    chassis.moveToPoint(54, 45, 2000, {}, false);

    chassis.moveToPoint(45, 45, 2000, {}, false);

    chassis.turnToPoint(35, 45, 2000, {}, false);
    chassis.moveToPoint(35, 45, 2000, {}, false);

    chassis.swingToHeading(210,lemlib::DriveSide::RIGHT, 2000, {
            .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE }, false);

    chassis.moveToPoint(22.5, 22.5, 2000, {}, false);
    chassis.moveToPoint(14, 14, 2000, {}, false);

    chassis.swingToHeading(90,lemlib::DriveSide::RIGHT, 2000, {
            .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE }, false);

    chassis.moveToPoint(60, 20, 2000, {}, false);
    chassis.turnToPoint(60, -8, 2000, {}, false);
    chassis.moveToPoint(60, -8, 2000, {}, false);

    // chassis.turnToPoint(0, 0, 2000, {}, false);
    //
    // chassis.moveToPoint(14, -15, 2000, {}, false);
    //
    // chassis.moveToPoint(45, -47, 2000, {.forwards=false}, false);
    // chassis.turnToPoint(67, -47, 2000, {}, false);
    // chassis.moveToPoint(55.5, -47, 2000, {}, false);
    // pros::delay(1000);
    // // pros::delay(1000);
    // chassis.moveToPoint(45, -47, 2000, {.forwards=false}, false);
    // // chassis.moveToPoint(54, -47, 2000, {}, false);
    // chassis.turnToPoint(0, -47, 2000, {}, false);
    // chassis.moveToPoint(32, -47, 2000, {}, false);

    // auto start_time = pros::millis();
    //
    // while (true) {
    //     if (pros::millis() - start_time > 1000) { break; }
    //     intake::set(intake::forwards);
    //     std::cout << to_in(RobotGetPose().x) << " " << to_in(RobotGetPose().y) << " "
    //               << to_stDeg(RobotGetPose().orientation) << std::endl;
    //     printf("cha: %f %f %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
    //     pros::delay(30);
    // }
    // std::cout << to_in(RobotGetPose().x) << " " << to_in(RobotGetPose().y) << " "
    //           << to_stDeg(RobotGetPose().orientation) << std::endl;
    // printf("cha: %f %f %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

    // chassis.turnToPoint(47,-47,2000);
    // chassis.moveToPoint(47,-47,2000);
    // chassis.turnToPoint(70,-47,2000);
    // chassis.moveToPoint(54,-47,2000);
}

} // namespace auton1
