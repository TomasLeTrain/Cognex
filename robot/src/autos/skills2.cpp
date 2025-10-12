/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "autos.h"
#include "blazing/utils.hpp"
#include "globals.h"
#include "pros/abstract_motor.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include <iostream>

// do not do anything outside here!

namespace skills2 {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

    // bool print_info = false;
    //
    // pros::Task smoother_task {[&] {
    // while (print_info) {
    //     int start_time = pros::millis();
    //     printf(
    //       "start generation\nstart distances\nend distances\nstart " "parti"
    //       "cles" "\n");
    //
    //     printf("%.1f %.1f %.1f\n",
    //            pf_motion_model.getPose().x.convert(in),
    //            pf_motion_model.getPose().y.convert(in),
    //            0.0);
    //     if(pf_model.getConfidence() != std::nullopt){
    //         printf("%.1f %.1f %.1f\n",
    //                 pf_model.getPose().x.convert(in),
    //                 pf_model.getPose().y.convert(in),
    //                 5.0);
    //     }
    //     printf("%.1f %.1f %.1f\n",
    //            smoother_model.getPose().x.convert(in),
    //            smoother_model.getPose().y.convert(in),
    //            10.0);
    //
    //     printf(
    //       "end particles\ntotal weight: 0, time taken: 30000, " "timestamp:"
    //       " %d\n", start_time);
    //     printf("things done:1,1,0,%d\n",16384);
    //     printf("prediction:%.1f,%.1f,%.1f\n",
    //            smoother_model.getPose().x.convert(in),
    //            smoother_model.getPose().y.convert(in),
    //            smoother_model.getPose().orientation.convert(deg));
    //     printf("end generation\n");
    //
    //     pros::delay(10);
    // }
    // }};

    // pose_getter = &smoother_model;

    // pros::delay(40);
    RobotSetPose(-63, -16, 0);
    pros::delay(40);

    // don't color sort
    intake::setColorSortEnabled(false);
    intake::set(intake::intake);

    // clear park
    mb.moveTo(-63.3, 16.5) | run;

    // pros::delay(1000);
    // left_motor_group.move(0);
    // right_motor_group.move(0);

    // chassis.turnToHeading(24,2000,{.minSpeed=20,.earlyExitRange=2});

    // motions::moveArc(20.5_in, 140, 28_in / sec,
    // lemlib::AngularDirection::CW_CLOCKWISE, 5000, {});
    // motions::moveArc(13.5_in, 130, 22_in / sec,
    // lemlib::AngularDirection::CW_CLOCKWISE, 2000, {.maxAccel=200_rpm},
    // false);
    mb.boomerang(-35, 31, 135).lead(0.6) | run;

    // left_motor_group.set_brake_mode_all(pros::MotorBrake::hold);
    // right_motor_group.set_brake_mode_all(pros::MotorBrake::hold);

    // move towards first corner
    // chassis.moveToPose(-29,31,145,2000,{.lead=0.5});

    // move towards center goal to score
    mb.moveTo(-10, 11).linear_clampMaxVoltage(0.3_volt) | async;

    pros::delay(2300);
    matchloader::set(true);
    async.wait();

    // score on center top goal
    pf_model.setDisabled(true);

    // color sort balls so we only score blue balls
    // intake::setColorSortEnabled(true);
    intake::setColorSortEnabled(false);
    auto_alliance = alliance_t::blue;

    // attempt to intake any balls still there
    pros::delay(400);

    intake::set(intake::scoring_middle);
    pros::delay(2300);

    // enable the particle filter now
    pf_model.setDisabled(false);

    intake::setColorSortEnabled(false);
    intake::set(intake::intake);
    matchloader::set(false);

    // swing to face second corner
    mb.turnTo(-23.3, -22.5)
        .direction(AngularDirection::LEFT)
        .radius(1.0)
        .angular_clampMinVoltage(0.3_volt) |
      run;

    // move to second corner
    // mb.moveTo(-30,-8, 2000, {.minSpeed=40,.earlyExitRange=5},
    // false);
    mb.turnTo(-40, -22).angular_clampMinVoltage(0.4_volt) | run;
    mb.moveTo(-40, -20) | run;

    mb.turnTo(-23, -22).angular_clampMinVoltage(0.4_volt) | run;
    mb.moveTo(-23, -22).linear_clampMaxVoltage(0.9_volt) | run;

    // turn to and go the third corner
    mb.turnTo(23.5, -23.5) | run;
    mb.moveTo(23.5, -23.5)
        .linear_clampMaxVoltage(0.7_volt)
        .linear_clampMinVoltage(0.2_volt) |
      run;

    matchloader::set(true);

    // go to matchloader
    // chassis.moveToPose(56.5,-44,90,3200,{ .lead=0.6 });
    mb.moveTo(44, -46.5) | run;
    mb.moveTo(57, -47.5) | run;

    // pf_model.setDisabled(true);
    pros::delay(1300);
    matchloader::set(false);
    // pf_model.setDisabled(false);

    // back up to go to goal
    mb.moveTo(50, -47.5).reverse().linear_clampMinVoltage(0.1_volt) | run;

    // turn to and score on
    // long goal
    mb.turnTo(0, -48.5) | run;

    // start priming intake
    // intake::set(intake::priming);

    mb.moveTo(28, -48.5) | run;

    intake::set(intake::unjam);
    pros::delay(400);

    // score all balls
    intake::setColorSortEnabled(false);
    // auto_alliance =
    // alliance_t::blue;
    intake::set(intake::scoring_long);

    // pf_model.setDisabled(true);
    pros::delay(4000);
    // pf_model.setDisabled(false);

    intake::setColorSortEnabled(false);
    // auto_alliance =
    // alliance_t::blue;
    intake::set(intake::intake);

    // turn away from goal
    // and turn without
    // touching it
    mb.turnTo(180 + 30).direction(AngularDirection::RIGHT) | run;

    mb.turnTo(40)
        .direction(AngularDirection::LEFT)
        .angular_clampMinVoltage(20_volt) |
      run;

    // move towards blue park
    mb.boomerang(61.3, -17.5, 0).lead(0.3).linear_clampMinVoltage(0.5_volt) |
      run;

    // clear blue park
    mb.moveTo(62, 15.5).linear_clampMinVoltage(0.5_volt) | run;

    // wait for mcl to work?
    RobotSetPose(63, 16, 0);

    // set pose to not be cooked
    pros::delay(1000);

    // get to matchloader
    mb.turnTo(42, 44) | run;
    mb.moveTo(42, 44) | run;
    mb.turnTo(67, 47) | run;

    matchloader::set(true);
    mb.moveTo(57, 47) | run;

    // pf_model.setDisabled(true);
    pros::delay(1300);
    matchloader::set(false);
    // pf_model.setDisabled(false);

    mb.moveTo(46.8, 46.7).reverse() | run;

    // turn to and go to fourth corner
    mb.turnTo(32.3, 31.5) | run;
    mb.moveTo(32.3, 31.5) | run;

    // score on center goal
    mb.moveTo(11.3, 11.7).linear_clampMaxVoltage(0.5_volt) | run;
    // mb.moveTo(11, -11.5, 2500, {.maxSpeed=80});
    pf_model.setDisabled(true);

    // intake::setColorSortEnabled(true);
    intake::setColorSortEnabled(false);
    auto_alliance = alliance_t::red;
    intake::set(intake::scoring_bottom);

    pros::delay(2300);
    pf_model.setDisabled(false);

    intake::setColorSortEnabled(false);
    // auto_alliance = alliance_t::blue;
    intake::set(intake::intake);

    // back up and go to third matchloader
    mb.moveTo(18, 19.5).reverse() | run;

    mb.turnTo(-51, 40) | run;

    // move to matchloader
    mb.moveTo(-50, 44) | run;
    mb.turnTo(-71, 46.7).angular_clampMaxVoltage(0.2_volt) | run;
    matchloader::set(true);
    mb.moveTo(-55, 46.7) | run;
    // pf_model.setDisabled(true);
    pros::delay(1800);
    matchloader::set(false);
    // pf_model.setDisabled(false);

    // back up, turn to and go to long goal
    mb.moveTo(-50, 46.7).reverse() | run;

    mb.turnTo(0, 46.7) | run;
    mb.moveTo(-32, 46.7) | run;
    // pf_model.setDisabled(true);

    intake::setColorSortEnabled(false);
    // auto_alliance = alliance_t::red;
    intake::set(intake::scoring_long);

    pros::delay(3600);
    // pf_model.setDisabled(false);
    intake::setColorSortEnabled(false);
    // auto_alliance = alliance_t::red;
    intake::set(intake::intake_disabled);

    // swing and run for the park
    mb.turnTo(-63, 0)
        .direction(AngularDirection::LEFT)
        .radius(1.0)
        .linear_clampMinVoltage(0.4_volt) |
      run;

    mb.moveTo(-63, 0).linear_clampMinVoltage(0.5_volt) | run;
}

} // namespace skills2
