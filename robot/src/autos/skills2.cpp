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
#include "systems/matchloader.h"
#include "motions/arc.h"

// do not do anything outside here!

namespace skills2 {

// you can add any variables / functions here

void run() {
    // do whatever you want here

    // bool print_info = false;
    //
    // pros::Task smoother_task {[&] {
    // while (print_info) {
    //     int start_time = pros::millis();
    //     printf(
    //       "start generation\nstart distances\nend distances\nstart " "parti" "cles" "\n");
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
    //       "end particles\ntotal weight: 0, time taken: 30000, " "timestamp:" " %d\n",
    //       start_time);
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
    RobotSetPose(-63_in, -16_in, 0);
    pros::delay(40);

    // don't color sort
    intake::setColorSortEnabled(false);
    intake::set(intake::intake);

    // clear park
    chassis.moveToPoint(-63.3, 16.5, 3000, {}, false);
    
    // pros::delay(1000);
    // left_motor_group.move(0);
    // right_motor_group.move(0);

    // chassis.turnToHeading(24,2000,{.minSpeed=20,.earlyExitRange=2},false);

    // motions::moveArc(20.5_in, 140, 28_in / sec, lemlib::AngularDirection::CW_CLOCKWISE, 5000, {},false);
    // motions::moveArc(13.5_in, 130, 22_in / sec, lemlib::AngularDirection::CW_CLOCKWISE, 2000, {.maxAccel=200_rpm}, false);
    chassis.moveToPose(-35,31,135,1800,{.lead=0.6},false);

    // left_motor_group.set_brake_mode_all(pros::MotorBrake::hold);
    // right_motor_group.set_brake_mode_all(pros::MotorBrake::hold);

    // move towards first corner
    // chassis.moveToPose(-29,31,145,2000,{.lead=0.5},false);

    // move towards center goal to score
    // chassis.moveToPoint(-13.5, 12.5, 2300, {.maxSpeed=40}, true);
    chassis.moveToPoint(-10, 11, 2300, {.maxSpeed=40}, true);

    pros::delay(2300);
    matchloader::set(true);
    chassis.waitUntilDone();

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

    pf_model.setDisabled(false);

    intake::setColorSortEnabled(false);
    intake::set(intake::intake);
    matchloader::set(false);

    // swing to face second corner
    chassis.swingToPoint(-23.3,-22.5,lemlib::DriveSide::LEFT, 1000, {
            .direction=lemlib::AngularDirection::CW_CLOCKWISE,
            .minSpeed=30,
            .earlyExitRange=10
            },false);

    // move to second corner
    // chassis.moveToPoint(-30,-8, 2000, {.minSpeed=40,.earlyExitRange=5}, false);
    chassis.turnToPoint(-40,-22, 2000, {.minSpeed=40,.earlyExitRange=5}, false);
    chassis.moveToPoint(-40,-20, 2000, {.minSpeed=1,.earlyExitRange=10}, false);

    chassis.turnToPoint(-23,-22, 2000, {.minSpeed=40,.earlyExitRange=5}, false);
    chassis.moveToPoint(-23,-22, 2000, {.maxSpeed=100}, false);

    // turn to and go the third corner
    chassis.turnToPoint(23.5,-23.5, 2000, {}, false);
    chassis.moveToPoint(23.5,-23.5, 2000, {.maxSpeed=80,.minSpeed=20,.earlyExitRange=4}, false);

    matchloader::set(true);

    // go to matchloader
    // chassis.moveToPose(56.5,-44,90,3200,{ .lead=0.6 },false);
    chassis.moveToPoint(44, -46.5, 2000, {}, false);
    chassis.moveToPoint(57, -47.5, 2000, {}, false);

    // pf_model.setDisabled(true);
    pros::delay(1300);
    matchloader::set(false);
    // pf_model.setDisabled(false);

    // back up to go to goal
    chassis.moveToPoint(50, -47.5, 2000, { .forwards=false,.minSpeed=14,.earlyExitRange=5 }, false);

    // turn to and score on long goal
    chassis.turnToPoint(0, -48.5, 2000, {}, false);

    // start priming intake
    // intake::set(intake::priming);

    chassis.moveToPoint(28, -48.5, 2000, {}, false);

    intake::set(intake::unjam);
    pros::delay(400);
    
    // score all balls
    intake::setColorSortEnabled(false);
    // auto_alliance = alliance_t::blue;
    intake::set(intake::scoring_long);

    // pf_model.setDisabled(true);
    pros::delay(4000);
    // pf_model.setDisabled(false);

    intake::setColorSortEnabled(false);
    // auto_alliance = alliance_t::blue;
    intake::set(intake::intake);

    // turn away from goal and turn without touching it
    chassis.swingToHeading(180+30,lemlib::DriveSide::RIGHT, 800, {
            .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
            .minSpeed=80,
            .earlyExitRange=20,
            },false);
    chassis.turnToHeading(40, 1000, {
            .direction=lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,
            .minSpeed=20,
            .earlyExitRange=10,
            },false);

    // move towards blue park
    chassis.moveToPose(61.3,-17.5,0 ,1500,{.lead=0.3,.minSpeed=60,.earlyExitRange=13},false);

    // clear blue park
    chassis.moveToPoint(62, 15.5, 3000, {.minSpeed=70,.earlyExitRange=5}, false);

    // wait for mcl to work?
    RobotSetPose(63_in, 16_in, 0);

    // set pose to not be cooked
    pros::delay(1000);

    // get to matchloader
    chassis.turnToPoint(42,44, 2000,{},false);
    chassis.moveToPoint(42,44, 2000,{},false);
    chassis.turnToPoint(67,47, 2000,{},false);

    matchloader::set(true);
    chassis.moveToPoint(57,47, 2000,{},false);

    // pf_model.setDisabled(true);
    pros::delay(1300);
    matchloader::set(false);
    // pf_model.setDisabled(false);

    chassis.moveToPoint(46.8, 46.7, 2000, {.forwards=false}, false);

    // turn to and go to fourth corner
    chassis.turnToPoint(32.3, 31.5, 2000, {}, false);
    chassis.moveToPoint(32.3, 31.5, 2000, {}, false);

    // score on center goal
    chassis.moveToPoint(11.3, 11.7, 3000, {.maxSpeed=80}, false);
    // chassis.moveToPoint(11, -11.5, 2500, {.maxSpeed=80}, false);
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
    chassis.moveToPoint(18, 19.5, 2000, {.forwards=false}, false);
    chassis.turnToPoint(-51, 40, 2000, {}, false);

    // move to matchloader
    chassis.moveToPoint(-50, 44, 2000, {}, false);
    chassis.turnToPoint(-71, 46.7, 2000, {.minSpeed=20,.earlyExitRange=10}, false);
    matchloader::set(true);
    chassis.moveToPoint(-55, 46.7, 2000, {}, false);
    // pf_model.setDisabled(true);
    pros::delay(1800);
    matchloader::set(false);
    // pf_model.setDisabled(false);

    // back up, turn to and go to long goal
    chassis.moveToPoint(-50,46.7,2000,{.forwards=false},false);
    chassis.turnToPoint(0,46.7,2000,{},false);
    chassis.moveToPoint(-32, 46.7,2000,{},false);
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
    chassis.swingToPoint(-63,0,lemlib::DriveSide::LEFT, 800, {
            .direction=lemlib::AngularDirection::CW_CLOCKWISE,
            .minSpeed=40,
            .earlyExitRange=15
            },false);
    chassis.moveToPoint(-63,0,2000,{.minSpeed=60},false);
    }

} // namespace auton1
