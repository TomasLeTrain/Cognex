/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "autos.h"
#include "globals.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include <iostream>

// do not do anything outside here!

namespace simple_auton {

// you can add any variables / functions here

void run() {
    // do whatever you want here

    // bool printing = false;
    //
    // pros::Task smoother_task {
    //     [&] {
    //         while (printing) {
    //             int start_time = pros::millis();
    //             printf("start generation\nstart distances\nend " "distances\nst"
    //                                                              "ar" "t " "par"
    //                                                                        "ti" "cl" "es" "\n");
    //
    //             printf("%.1f %.1f %.1f\n",
    //                    pf_motion_model.getPose().x.convert(in),
    //                    pf_motion_model.getPose().y.convert(in),
    //                    0.0);
    //             printf("%.1f %.1f %.1f\n",
    //                    pf_model.getPose().x.convert(in),
    //                    pf_model.getPose().y.convert(in),
    //                    5.0);
    //             printf("%.1f %.1f %.1f\n",
    //                    smoother_model.getPose().x.convert(in),
    //                    smoother_model.getPose().y.convert(in),
    //                    10.0);
    //
    //             printf(
    //               "end particles\ntotal weight: 0, time taken: 30000, " "timest" "amp:" " %d\n",
    //               start_time);
    //             printf("things done:1,1,0,%d\n", 16384);
    //             printf("prediction:%.1f,%.1f,%.1f\n",
    //                    smoother_model.getPose().x.convert(in),
    //                    smoother_model.getPose().y.convert(in),
    //                    smoother_model.getPose().orientation.convert(deg));
    //             printf("end generation\n");
    //
    //             pros::delay(20);
    //         }
    //     }
    // };

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    double start_angle = bl ? 245 : 2 * 270 - 245;

    RobotSetPose(47.15_in, -11.7_in * l, start_angle);
    pros::delay(200);

    // intake::setColorSortEnabled(true);
    intake::set(intake::intake);

    // std::cout << RobotGetPose().x << " " << RobotGetPose().y << " " <<
    // RobotGetPose().orientation << std::endl; printf("chassis: %f %f %f\n",
    // chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

    // chassis.turnToHeading(0,2000,{},false);
    // chassis.turnToHeading(0,9000,{},false);
    pf_model.setDisabled(true);
    chassis.moveToPoint(22, -22 * l, 2000, {.maxSpeed=40}, false);

    chassis.turnToPoint(0, 0 * l, 2000, {}, false);

    if(bl){
        matchloader::set(true);
    }

    // pros::delay(100);

    if(bl){
        chassis.moveToPoint(12, -12.5 * l, 2500, {.maxSpeed=80}, false);
        intake::set(intake::scoring_middle);
    }else{
        chassis.moveToPoint(12, -12.5 * l, 2500, {.maxSpeed=80}, false);
        intake::set(intake::scoring_bottom);
    }

    // make sure its down
    // matchloader::set(false);

    pros::delay(2000);
    intake::set(intake::intake);

    pf_model.setDisabled(false);

    chassis.moveToPoint(40, -47.5 * l, 2000, { .forwards = false,.maxSpeed=65 }, false);
    matchloader::set(true);

    chassis.turnToPoint(67, -47 * l, 2000, {.maxSpeed=65}, false);
    intake::set(intake::intake_slow_bottom);

    chassis.moveToPoint(56.5, -47 * l, 2000, {.maxSpeed=40}, false);

    // matchload
    pros::delay(400);

    matchloader::set(false);

    chassis.moveToPoint(45, -47 * l, 2000, { .forwards = false,.maxSpeed=65 }, false);
    chassis.turnToPoint(0, -47 * l, 2000, {.maxSpeed=65}, false);
    // intake::set(intake::priming);
    chassis.moveToPoint(28, -47 * l, 1800, {.maxSpeed=75}, false);
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

    // chassis.turnToPoint(47,-47,2000);
    // chassis.moveToPoint(47,-47,2000);
    // chassis.turnToPoint(70,-47,2000);
    // chassis.moveToPoint(54,-47,2000);
}

} // namespace simple_auton
