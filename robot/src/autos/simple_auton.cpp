/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "apis.h"
//
#include "autos.h"
#include "globals.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "units/Angle.hpp"
#include <iostream>

// do not do anything outside here!

namespace simple_auton {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

    // bool printing = false;
    //
    // pros::Task smoother_task {
    //     [&] {
    //         while (printing) {
    //             int start_time = pros::millis();
    //             printf("start generation\nstart distances\nend "
    //             "distances\nst"
    //                                                              "ar" "t "
    //                                                              "par"
    //                                                                        "ti"
    //                                                                        "cl"
    //                                                                        "es"
    //                                                                        "\n");
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
    //               "end particles\ntotal weight: 0, time taken: 30000, "
    //               "timest" "amp:" " %d\n", start_time);
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

    double start_angle = bl ? 205 : 360 - 205;

    RobotSetPose(47.15, -11.7 * l, start_angle);
    pros::delay(200);

    // intake::setColorSortEnabled(true);
    intake::set(intake::intake);

    // std::cout << RobotGetPose().x << " " << RobotGetPose().y << " " <<
    // RobotGetPose().orientation << std::endl; printf("chassis: %f %f %f\n",
    // chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);

    pf_model.setDisabled(true);
    mb.moveTo(22, -22 * l).drive_maxVolt(0.45_volt) | run;

    mb.turnTo(0, 0) | run;

    if (bl) {
        matchloader::set(matchloader::active);
    }

    // pros::delay(100);

    if (bl) {
        mb.moveTo(11.5, -12 * l).drive_maxVolt(0.7_volt) | run;
        mb.turnTo(0, 0 * l) | run;
        intake::set(intake::scoring_middle);
    } else {
        mb.moveTo(11.8, -12.8 * l).drive_maxVolt(0.5_volt) | run;
        mb.turnTo(0, 0 * l) | run;
        intake::set(intake::scoring_bottom);
    }

    // make sure its down
    // matchloader::set(matchloader::inactive);

    pros::delay(1200);
    intake::set(intake::intake);

    pf_model.setDisabled(false);

    mb.moveTo(40, -47.5 * l).reverse().drive_maxVolt(0.5_volt) | run;

    matchloader::set(matchloader::active);

    mb.turnTo(67, -47 * l).turn_maxVolt(0.8_volt) | run;
    intake::set(intake::intake_slow_bottom);

    mb.moveTo(56.5, -47 * l).drive_maxVolt(0.8_volt) | run;

    // matchload
    pros::delay(300);

    mb.moveTo(45, -47 * l).reverse().drive_maxVolt(0.5_volt) | run;

    mb.turnTo(0, -47 * l).turn_maxVolt(0.5_volt) | run;
    matchloader::set(matchloader::inactive);
    intake::set(intake::priming);

    mb.moveTo(28, -47 * l).drive_maxVolt(0.6_volt) | run;

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

} // namespace simple_auton
