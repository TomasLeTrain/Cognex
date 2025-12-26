#include "apis.h"
//
#include "auton_globals.h"
#include "autos.h"
#include "blazing/utils.hpp"
#include "globals/blazing_globals.h"
#include "globals/config.h"
#include "globals/vexmaps_globals.h"
#include "pros/abstract_motor.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "vexmaps/mcl/distance_model.hpp"
#include <functional>
#include <tuple>

// do not do anything outside here!

namespace sunshine_skills {

// void first_matchloader_mp();
// void chained_first_matchloader(Length match1);

// you can add any variables / functions here

void run_auton() {
    // TODO: make distance autmatic
    auto make_machloader_point = [&](units::V2FPosition target,
                                     Length distance) -> units::V2FPosition {
        auto difference = (target - RobotGetPose()).normalize() *
                          (RobotGetPose().distanceTo(target) - distance);
        ;
        auto new_point = RobotGetPose() + difference;

        return new_point;
    };

    auto closeEnough = [](units::V2Position target,
                          Length threshold) -> std::function<bool()> {
        return [target, threshold] -> bool {
            return RobotGetPose().distanceTo(target) < threshold;
        };
    };

    auto start_time = now();

    units::V2FPosition target_point;

    intake::in();

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    Length match1 = normal_match;
    Length match2 = normal_match;
    Length match3 = -normal_match;
    Length match4 = -normal_match;

    units::V2Position centerBallOne = { -24_in, 24_in };

    units::V2Position centerTopGoalFirst = { -8.25_in, 6.8_in };
    units::V2Position centerTopGoalSecond = { 0_in, 0_in };
    units::V2Position centerBottomGoalFirst = { 12_in, 12_in };
    units::V2Position centerBottomGoalSecond = { -12_in, -11_in };

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(inactive);

    RobotSetPose(-47.2, 14.8, 0);
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // only intake bottom balls to save time
    // intake::set(intake::intake_bottom_balls);
    intake::setColorSortEnabled(false);
    // intake::setColorSortEnabled(true);
    // auto_alliance = alliance_t::red;
    // intake::in();
    intake::set(intake::intake_bottom_balls);

    // mb.moveTo(-25, 23.4).drive_maxVolt(0.7_volt).timeout(0.4_sec) | run;
    mb.moveTo(-31, 21.4).drive_maxVolt(0.3_volt) | run;
    intake::set(intake::intake_disabled);
    mb.moveTo(-25, 23.4) | run;
    // color sort balls here
    // chain.waitUntil(closeEnough({ -25_in, 23.4_in }, 10_in));
    // matchloader::down();
    // pros::delay(300);

    mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | chain;
    mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
        .k_lat(0.3)
        .drive_maxVolt(0.5_volt)
        .executeBeforeMotion([] {
            matchloader::up();
        }) |
      chain;

    chain.waitUntil(closeEnough(centerTopGoalFirst, 6_in));
    controller.rumble(".");
    // outtake slightly in case first ball is stuck
    intake::out();
    pros::delay(200);
    // intake::set(intake::scoring_middle_bottom_balls);
    intake::set(intake::scoring_middle_bottom_balls);
    // start_time = now();
    pros::delay(2000);

    // wait for 3 seconds or until it detects its gonna score a blue ball
    // while (!timeoutDone(3000_msec, start_time) ||
    //        intake::getMiddleDetectedColor() == alliance_t::blue) {
    //     pros::delay(10);
    // }

    // exit any motions if the are somehow still executing
    chain.exitAll();
    // matchloader::up();

    // move back

    mb.moveTo(-48, match1).reverse().only_y(true) | run;
    // its a run here, so we can do these things
    intake::in();
    intake::setColorSortEnabled(false);
    matchloader::down();

    mb.turnTo(180) | chain;
    mb.moveTo(-60.5, match1).drive_maxVolt(0.5_volt) | chain;
    chain.wait();
    pros::delay(1500);

    // mb.arc(330, -1.0).direction(AngularDirection::RIGHT).reverse() | run;

    mb.moveTo(-25, 58).reverse().executeAfterMotion([] {
        // intake::set(intake::intake_disabled);
        matchloader::up();
    }) |
      chain;
    // go to other side
    mb.moveTo(25, 58).reverse() | chain;
    // get on same y
    mb.moveTo(43, long_goal + 0.5_in).reverse().only_y(true) | chain;

    // turn to goal, reversed
    mb.turnTo(0_in, long_goal).reverse() | chain;
    chain.wait();

    // go to goal, reversed
    mb.moveTo(23_in, long_goal).reverse().timeout(1.4_sec).k_lat(0.0) | async;
    async.waitUntil(closeEnough({ 31_in, long_goal }, 3_in));

    intake::score_long();

    // wait for 2 seconds
    pros::delay(500);
    matchloader::down();
    pros::delay(2000);

    // exit any motions if the are somehow still executing
    async.exitAll();

    // mb.moveTo(55, match2).drive_maxVolt(0.5_volt).executeAfterMotion([] {
    //     intake::in();
    // }) |
    //   run;
    mb.turnTo(60, match2) | chain;
    mb.moveTo(60, match2).drive_maxVolt(0.5_volt) | chain;

    // let it score last one
    pros::delay(300);
    // outake a bit while going, in case something is stuck
    intake::out();
    pros::delay(200);
    intake::in();

    chain.wait();
    pros::delay(1700);

    // go to goal to score again
    mb.moveTo(24_in, long_goal).reverse().timeout(1.3_sec).k_lat(0.3) | async;

    async.waitUntil(closeEnough({ 32_in, long_goal }, 4_in));

    intake::score_long();

    pros::delay(500);
    matchloader::up();
    pros::delay(1300);

    // exit any motions if the are somehow still executing
    async.exitAll();

    mb.boomerang(62, 19, 270).lead(0.3).drive_maxVolt(0.5_volt) | chain;

    // let it score last one
    pros::delay(400);
    // outake a bit while going, in case something is stuck
    intake::out();

    chain.wait();
    intake::in();

    // mb.boomerang(63.755, 20.904, 270).lead(0.3) | run;

    // disable horizontal odom
    horizontal_tracker.setDisabled(true);
    odom_retract::retractOdom();

    // here the thresholds likely need to be bigger for reseting to happen
    // we later reset them back to what they should be
    setMaxDistanceThresholdAll(10_in);
    setSmootherAlphas(std::nullopt, smoother_config.alpha_y * 2.0);

    drivetrain.moveTank(0.125_volt, 0.2_volt);
    // move closer to the park slowly, also gives time for accurate start roll
    pros::delay(200);

    // roll is hopefully accurate here
    double start_roll = imu.get_roll();
    std::cout << "start roll: " << start_roll << std::endl;
    pros::delay(400);

    // get over first part of park
    //
    drivetrain.moveTank(0.4_volt, 0.5_volt);
    pros::delay(1500);
    drivetrain.moveTank(0.2_volt, 0.3_volt);
    matchloader::down();
    pros::delay(800);

    // enable odom again
    horizontal_tracker.setDisabled(false);
    odom_retract::lowerOdom();
    pros::delay(200);

    // the threshold for distance is still pretty big as localization has to be
    // pretty good
    //
    // explode center balls
    // matchloader::down();
    mb.turnTo(180) | run;
    // reset position to guarantee its not wrong at all
    LaserResets({ &back_laser_model, &left_laser_model });

    mb.moveTo(26, -23.6).drive_maxVolt(0.5_volt) | run;
    matchloader::up();

    // reset the max distance as we hope we have the right location
    resetMaxDistanceThresholdAll();
    resetSmootherConfig();

    // go to bottom goal
    mb.turnTo(centerTopGoalSecond.x, centerTopGoalSecond.y) | run;
    mb.distanceAtHeading(26_in).timeout(1.5_sec) | run;
    // mb.moveTo(centerTopGoalSecond.x, centerTopGoalSecond.y).k_lat(0.0) | run;

    chain.waitUntil(closeEnough({ 9_in, 9_in }, 3_in));

    intake::set(intake::scoring_middle);
    // start_time = now();

    pros::delay(4000);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    setSmootherAlphas(smoother_config.alpha_x * 2.0,
                      smoother_config.alpha_y * 2.0);
    mb.moveTo(48, match3).reverse().drive_maxVolt(0.4_volt) | run;
    resetSmootherConfig();
    matchloader::down();
    mb.turnTo(60, match3) | chain;
    mb.moveTo(60.5, match3).drive_maxVolt(0.5_volt) | chain;
    chain.wait();
    pros::delay(2000);

    mb.moveTo(25, -58).reverse().executeAfterMotion([] {
        matchloader::up();
    }) |
      chain;
    // go to other side
    mb.moveTo(-25, -58).reverse().executeBeforeMotion([] {
        setSmootherAlphas(smoother_config.alpha_x * 2.0,
                          smoother_config.alpha_y * 2.0);
    }) |
      chain;
    // get on same y
    mb.moveTo(-46, -long_goal).reverse() | chain;

    // turn to goal, reversed
    mb.turnTo(-24_in, -long_goal).reverse() | chain;

    chain.wait();
	resetSmootherConfig();

    // go to goal, reversed
    mb.moveTo(-24_in, -long_goal - 0.5_in)
        .reverse()
        .timeout(1.3_sec)
        .k_lat(0.0) |
      async;
    // wait until its close to goal
    async.waitUntil(closeEnough({ -32_in, -long_goal }, 4_in));

    intake::score_long();

    // wait for 2 seconds
    pros::delay(500);
    matchloader::down();
    pros::delay(1400);

    // exit any motions if the are somehow still executing
    async.exitAll();

    mb.turnTo(-60, match4) | chain;
    mb.moveTo(-60, match4).drive_maxVolt(0.5_volt) | chain;

    pros::delay(500);
    intake::in();
    chain.wait();
    pros::delay(2000);

    mb.moveTo(-24_in, -long_goal).reverse().timeout(1.3_sec).k_lat(0.0) | async;
    async.waitUntil(closeEnough({ -32_in, -long_goal }, 4_in));

    intake::score_long();

    pros::delay(1000);
    matchloader::up();
    pros::delay(1000);

    async.exitAll();

    // mb.moveTo(-34.687, -33.23) | chain;
    // mb.turnTo(centerBottomGoalSecond.x, centerBottomGoalSecond.y) | chain;
    // auto point4 = chain.getCurrentIndex();
    // mb.moveTo(centerBottomGoalSecond.x, centerBottomGoalSecond.y)
    //     .k_lat(0.3)
    //     .drive_maxVolt(0.5_volt) |
    //   chain;
    // chain.waitUntilIndex(point4);
    // chain.waitUntil(closeEnough({ -24_in, -24_in }, 5_in));
    // matchloader::down();
    // pros::delay(400);
    // matchloader::up();
    // chain.waitUntil(closeEnough(centerBottomGoalSecond, 5_in));
    //
    // intake::score_bottom();
    // pros::delay(1400);
    //
    // // exit any motions if the are somehow still executing
    // chain.exitAll();

    // mb.moveTo(-52.373, -25).reverse() | chain;
    // mb.turnTo(110).direction(AngularDirection::LEFT) | chain;
    mb.boomerang(-62, -20, 90).lead(0.3).drive_maxVolt(0.5_volt) | run;
    // mb.turnTo(275) | chain;

    // reset start roll again
    // start_roll = imu.get_roll();

    horizontal_tracker.setDisabled(true);
    odom_retract::retractOdom();
    pros::delay(300);

    // go into park
    // drivetrain.moveTank(0.4_volt, 0.45_volt);
    // drivetrain.moveTank(0.6_volt, 0.65_volt);
    // // while (std::abs(start_roll - imu.get_roll()) < 4) {
    // //     std::cout << std::abs(start_roll - imu.get_roll()) << std::endl;
    // //     controller.rumble(".");
    // //     pros::delay(20);
    // // }
    // pros::delay(750);

    drivetrain.moveTank(0.125_volt, 0.2_volt);
    // move closer to the park slowly, also gives time for accurate start roll
    pros::delay(200);

    // roll is hopefully accurate here
    // double start_roll = imu.get_roll();
    // std::cout << "start roll: " << start_roll << std::endl;
    pros::delay(400);

    // get over first part of park
    //
    drivetrain.moveTank(0.4_volt, 0.5_volt);
    pros::delay(1000);
    // drivetrain.moveTank(0.2_volt, 0.3_volt);
    // matchloader::down();
    // pros::delay(800);

    // stop the robot
    drivetrain.moveTank(0.0_volt, 0.0_volt);
}

} // namespace sunshine_skills
