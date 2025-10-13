/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "apis.h"
//
#include "autos.h"
#include "systems/intake.h"
#include <iostream>

// do not do anything outside here!

namespace simple_other_goal_auton {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

    pros::Task smoother_task { [&] {
        while (true) {
            int start_time = pros::millis();
            printf("start generation\nstart distances\nend distances\nstart "
                   "parti"
                   "cles"
                   "\n");

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

            printf("end particles\ntotal weight: 0, time taken: 30000, "
                   "timestamp:"
                   " %d\n",
                   start_time);
            printf("things done:1,1,0,%d\n", 16384);
            printf("prediction:%.1f,%.1f,%.1f\n",
                   smoother_model.getPose().x.convert(in),
                   smoother_model.getPose().y.convert(in),
                   smoother_model.getPose().orientation.convert(deg));
            printf("end generation\n");

            pros::delay(20);
        }
    } };

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    double start_angle = bl ? 205.915 : 360 - 205.915;

    RobotSetPose(47.489, -11.751 * l, start_angle);
    intake::set(intake::intake);

    // std::cout << RobotGetPose().x << " " << RobotGetPose().y << " " <<
    // RobotGetPose().orientation << std::endl; printf("chassis: %f %f %f\n",
    // chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

    // chassis.turnToHeading(0,2000,{});
    // chassis.turnToHeading(0,9000,{});
    mb.moveTo(22.8, -23.3 * l) | run;

    mb.turnTo(0, 0) | run;

    mb.moveTo(14, -15 * l) | run;
    intake::set(intake::scoring_middle);
    pros::delay(3000);
    intake::set(intake::intake);

    mb.moveTo(45, -47 * l).reverse() | run;
    mb.turnTo(67, -47 * l) | run;
    mb.moveTo(55.5, -47 * l) | run;
    // matchload
    pros::delay(3000);
    mb.moveTo(45, -47 * l).reverse() | run;

    mb.turnTo(45, 47 * l) | run;
    mb.moveTo(45, 47 * l) | run;
    mb.turnTo(0, 47 * l) | run;

    mb.moveTo(32, 47 * l) | run;
    intake::set(intake::scoring_long);

    // auto start_time = pros::millis();
    //
    // while (true) {
    //     if (pros::millis() - start_time > 1000) { break; }
    //     intake::set(intake::forwards);
    //     std::cout << to_in(RobotGetPose().x) << " " <<
    //     to_in(RobotGetPose().y) << " "
    //               << to_stDeg(RobotGetPose().orientation) << std::endl;
    //     printf("cha: %f %f %f\n", chassis.getPose().x, chassis.getPose().y,
    //     chassis.getPose().theta); pros::delay(30);
    // }
    // std::cout << to_in(RobotGetPose().x) << " " << to_in(RobotGetPose().y) <<
    // " "
    //           << to_stDeg(RobotGetPose().orientation) << std::endl;
    // printf("cha: %f %f %f\n", chassis.getPose().x, chassis.getPose().y,
    // chassis.getPose().theta);

    // mb.turnTo(47,-47,2000);
    // mb.moveTo(47,-47,2000);
    // mb.turnTo(70,-47,2000);
    // mb.moveTo(54,-47,2000);
}

} // namespace simple_other_goal_auton
