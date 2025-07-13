/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add it to "autos.h"
 */

#include "autos.h"
#include <iostream>
#include "systems/intake.h"

// do not do anything outside here!

namespace auton1 {

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

    RobotSetPose(47.489_in, -11.751_in, 244.085);

    std::cout << RobotGetPose().x << " " << RobotGetPose().y << " " << RobotGetPose().orientation << std::endl;
    printf("chassis: %f %f %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

    // chassis.turnToHeading(0,2000,{},false);
    // chassis.turnToHeading(0,9000,{},false);
    chassis.moveToPoint(22.8, -23.3, 2000, {}, false);

    chassis.turnToPoint(0, 0, 2000, {}, false);

    chassis.moveToPoint(14, -15, 2000, {}, false);

    chassis.moveToPoint(45, -47, 2000, {.forwards=false}, false);
    chassis.turnToPoint(67, -47, 2000, {}, false);
    chassis.moveToPoint(55.5, -47, 2000, {}, false);
    pros::delay(1000);
    chassis.moveToPoint(45, -47, 2000, {.forwards=false}, false);
    chassis.turnToPoint(0, -47, 2000, {}, false);
    chassis.moveToPoint(32, -47, 2000, {}, false);

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
