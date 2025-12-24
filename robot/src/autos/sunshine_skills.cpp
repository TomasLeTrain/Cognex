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

// do not do anything outside here!

namespace sunshine_skills {

// void first_matchloader_mp();
// void chained_first_matchloader(Length match1);

// you can add any variables / functions here

void run_auton() {
    RobotSetPose(-63.5, 16.2, 90);
    intake::setColorSortEnabled(false);

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

    units::V2Position centerTopGoalFirst = { -12_in, 10.8_in };
    units::V2Position centerTopGoalSecond = { 12.25_in, -10.8_in };
    units::V2Position centerBottomGoalFirst = { 13_in, 13_in };
    units::V2Position centerBottomGoalSecond = { -13_in, -13_in };

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(inactive);

    RobotSetPose(-47.2, 14.8, 0);

    // only intake bottom balls to save time
    intake::set(intake::intake_bottom_balls);

    mb.moveTo(-25, 23.6).drive_maxVolt(0.3_volt) | chain;

    mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | chain;
    mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
        .k_lat(0.3)
        .drive_maxVolt(0.5_volt) |
      chain;

    chain.waitUntil(closeEnough({ -27.7_in, 21.23_in }, 5_in));
    matchloader::down();

    chain.waitUntil(closeEnough(centerTopGoalFirst, 8_in));
    matchloader::up();
    chain.waitUntil(closeEnough(centerTopGoalFirst, 5_in));
    controller.rumble(".");
    // only balls on the bottom, so only need to score those
    intake::set(intake::scoring_middle_bottom_balls);
    start_time = now();

    // wait for 3 seconds or until it detects its gonna score a blue ball
    while (!timeoutDone(3000_msec, start_time) ||
           intake::getMiddleDetectedColor() == alliance_t::blue) {
        pros::delay(10);
    }

    // exit any motions if the are somehow still executing
    chain.exitAll();
    // matchloader::up();

    // move back

    mb.moveTo(-48, match1).reverse() | run;
    intake::in();
    mb.turnTo(-60, match1) | chain;
    mb.moveTo(-61, match1).drive_maxVolt(0.5_volt) | chain;
    chain.wait();
    pros::delay(2000);

    mb.arc(345, 1.0).direction(AngularDirection::LEFT).reverse() | chain;

    mb.moveTo(-25, 34).reverse() | chain;
    // go to other side
    mb.moveTo(25, 34).reverse() | chain;
    // get on same y
    mb.moveTo(46, long_goal).reverse() | chain;
    auto point1 = chain.getCurrentIndex();

    // turn to goal, reversed
    mb.turnTo(24_in, long_goal).reverse() | chain;
    // go to goal, reversed
    mb.moveTo(24_in, long_goal).reverse().timeout(1.2_sec).k_lat(0.0) | chain;
    // wait until we are in motion we would like it to trigger in
    chain.waitUntilIndex(point1);
    // wait until its close to goal
    chain.waitUntil(closeEnough({ 32_in, long_goal }, 3_in));

    intake::score_long();

    // wait for 2 seconds
    pros::delay(500);
    matchloader::down();
    pros::delay(1000);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    mb.turnTo(60, match2).executeAfterMotion([] {
        intake::in();
    }) |
      chain;
    mb.moveTo(61, match2).drive_maxVolt(0.5_volt) | chain;
    chain.wait();
    pros::delay(1500);

    // go to goal to score again
    mb.moveTo(24_in, long_goal).reverse().timeout(1.2_sec).k_lat(0.0) | chain;

    chain.waitUntil(closeEnough({ 32_in, long_goal }, 4_in));

    intake::score_long();

    pros::delay(500);
    matchloader::up();
    pros::delay(1000);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    mb.moveTo(30, 47.2) | chain;
    mb.moveTo(24, 23.6) | chain;
    mb.turnTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y) | chain;
    auto point2 = chain.getCurrentIndex();
    mb.moveTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y) | chain;

    chain.waitUntil(closeEnough({ 26.763_in, 34.64_in }, 5_in));
    matchloader::down();

    chain.waitUntilIndex(point2);
    matchloader::up();

    chain.waitUntil(closeEnough(centerBottomGoalFirst, 5_in));
    intake::score_bottom();

    pros::delay(1500);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    mb.moveTo(52.373, 30).reverse() | chain;
    mb.boomerang(62, 20, 270) | chain;
    // mb.turnTo(275) | chain;

    chain.wait();
    intake::in();

    // mb.boomerang(63.755, 20.904, 270).lead(0.3) | run;

    // disable horizontal odom
    horizontal_tracker.setDisabled(true);
    odom_retract::retractOdom();

    // here the thresholds likely need to be bigger for reseting to happen
    // we later reset them back to what they should be
    setMaxDistanceThresholdAll(10_in);

    drivetrain.moveTank(0.125_volt, 0.2_volt);
    // move closer to the park slowly, also gives time for accurate start roll
    pros::delay(400);

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
    pros::delay(3000);
    // drivetrain.moveTank(0.5_volt, 0.4_volt);
    //
    // // positive only when going up at the park
    // while (std::abs(imu.get_roll() - start_roll) < 4) {
    //     std::cout << imu.get_roll() - start_roll << std::endl;
    //     controller.rumble(".");
    //     pros::delay(20);
    // }
    // std::cout << "done" << std::endl;
    // pros::delay(100);
    // // hopefully got some initial balls
    //
    // // go fast again to avoid getting stuck
    // drivetrain.moveTank(0.3_volt, 0.25_volt);
    // // go through rest of the balls
    // pros::delay(300);
    // // now in park, go slowish
    // drivetrain.moveTank(0.2_volt, 0.15_volt);
    // pros::delay(400);
    // matchloader::down();
    // pros::delay(400);
    //
    // // get out of park
    // drivetrain.moveTank(0.5_volt, 0.4_volt);
    //
    // // when all but one wheel are over park
    // // negative while going down in the motion
    // while (std::abs(imu.get_roll() - start_roll) < 8) {
    //     std::cout << imu.get_roll() - start_roll << std::endl;
    //     controller.rumble(".");
    //     pros::delay(20);
    // }
    // std::cout << "done" << std::endl;
    //
    // // give time to finally get out of park
    // pros::delay(400);
    // over park now, can continue with run

    // enable odom again
    horizontal_tracker.setDisabled(false);
    odom_retract::lowerOdom();

    // the threshold for distance is still pretty big as localization has to be
    // pretty good
    //
    // explode center balls
    // matchloader::down();
    mb.moveTo(24, -24).drive_maxVolt(0.5_volt) | run;
    matchloader::up();

    // reset the max distance as we hope we have the right location
    resetMaxDistanceThresholdAll();

    // go to bottom goal
    mb.turnTo(centerTopGoalSecond.x, centerTopGoalSecond.y) | chain;
    mb.moveTo(centerTopGoalSecond.x, centerTopGoalSecond.y) | chain;

    chain.waitUntil(closeEnough(centerTopGoalSecond, 4_in));
    // only balls on the bottom, so only need to score those
    intake::set(intake::scoring_middle);
    start_time = now();

    pros::delay(4000);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    mb.moveTo(48, match3).reverse() | run;
    matchloader::down();
    mb.turnTo(60, match3) | chain;
    mb.moveTo(60, match3) | chain;
    chain.wait();
    pros::delay(2000);

    mb.moveTo(25, -34).reverse() | chain;
    // go to other side
    mb.moveTo(-25, -34).reverse() | chain;
    // get on same y
    mb.moveTo(-46, -long_goal).reverse() | chain;
    auto point3 = chain.getCurrentIndex();

    // turn to goal, reversed
    mb.turnTo(24_in, -long_goal).reverse() | chain;
    // go to goal, reversed
    mb.moveTo(24_in, -long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | chain;
    // wait until we are in motion we would like it to trigger in
    chain.waitUntilIndex(point3);
    // wait until its close to goal
    chain.waitUntil(closeEnough({ 32_in, -long_goal }, 4_in));

    intake::score_long();

    // wait for 2 seconds
    pros::delay(500);
    matchloader::down();
    pros::delay(1000);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    mb.turnTo(-60, match4) | chain;
    mb.moveTo(-60, match4) | chain;
    chain.wait();
    pros::delay(1500);

    chain.waitUntil(closeEnough({ -32_in, -long_goal }, 4_in));

    intake::score_long();

    pros::delay(1500);

    chain.exitAll();

    mb.moveTo(-34.687, -33.23) | chain;
    mb.turnTo(centerBottomGoalSecond.x, centerBottomGoalSecond.y) | chain;
    auto point4 = chain.getCurrentIndex();
    mb.moveTo(centerBottomGoalSecond.x, centerBottomGoalSecond.y).k_lat(0.3) |
      chain;
    chain.waitUntilIndex(point4);
    chain.waitUntil(closeEnough({ -24_in, -24_in }, 5_in));
    chain.waitUntil(closeEnough(centerBottomGoalSecond, 5_in));

    intake::score_bottom();
    pros::delay(1400);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    mb.moveTo(-62.529_in, -23_in).reverse() | chain;
    mb.turnTo(-66.507_in, 0_in) | chain;

    // reset start roll again
    start_roll = imu.get_roll();

    horizontal_tracker.setDisabled(true);
    odom_retract::retractOdom();

    // go into park
    drivetrain.moveTank(0.6_volt, 0.65_volt);
    while (std::abs(start_roll - imu.get_roll()) < 4) {
        std::cout << std::abs(start_roll - imu.get_roll()) << std::endl;
        controller.rumble(".");
        pros::delay(20);
    }
    pros::delay(400);

    // stop the robot
    drivetrain.moveTank(0.0_volt, 0.0_volt);
}

} // namespace sunshine_skills
