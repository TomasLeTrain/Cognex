#include "apis.h"
//
#include "auton_globals.h"
#include "autos.h"
#include "globals/blazing_globals.h"
#include "globals/config.h"
#include "globals/vexmaps_globals.h"
#include "pros/abstract_motor.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"
#include "units/Vector2D.hpp"
#include "vexmaps/mcl/distance_model.hpp"

// do not do anything outside here!

namespace sunshine_skills {

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

    units::V2FPosition target_point;

    intake::set(intake::intake);

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    Length match1 = -normal_match;
    Length match2 = -normal_match;
    Length match3 = normal_match;
    Length match4 = normal_match;

    units::V2Position centerBallOne = { -24_in, 24_in };

    units::V2Position centerTopGoal = { -13_in, 12_in };
    units::V2Position centerBottomGoal = { 13_in, 12_in };

    // move away from park
    mb.moveTo(-36_in, 36_in) | run;

    // mb.turnTo(centerBallOne.x, centerBallOne.y) | run;
    // mb.moveTo(centerBallOne.x, centerBallOne.y) | run;

    // move to top center goal
    mb.turnTo(centerTopGoal.x, centerTopGoal.y) | run;
    mb.moveTo(centerTopGoal.x, centerTopGoal.y).k_lat(0.3) | run;

    // move back
    drivetrain.moveTank(-0.6_volt, -0.5_volt);
    pros::delay(300);

    // move to center balls 2
    mb.moveTo(-24_in, -24_in)
        .k_lat(0.3)
        .drive_ErrorTolerance(7_in)
        .drive_ToleranceDuration(0_sec) |
      run;

    // boomerang to matchloader 1
    mb.boomerang(-58_in, match1, 180).drive_maxVolt(0.4_volt) | run;

    // move to goal
    mb.moveTo(-24_in, -long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;

    // move towards balls
    drivetrain.moveTank(1.0_volt, -1.0_volt);
    pros::delay(400);

    // get balls
    mb.moveTo(24, -24) | run;

    // get close to matchload 2
    mb.moveTo(48, -48) | run;

    // turn to matchload 2
    mb.turnTo(70, match2) | run;

    target_point = make_machloader_point({ 67.71_in, match2 }, 7_in);
    mb.moveTo(target_point.x, target_point.y) | run;

    // move to goal
    mb.moveTo(24_in, -long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;

    // simulate going over park
    mb.moveTo(48_in, 24_in)
        .drive_ErrorTolerance(7_in)
        .drive_ToleranceDuration(0_sec) |
      run;

    // explode center balls
    mb.moveTo(24, 24) | run;

    // go to bottom goal
    mb.turnTo(centerBottomGoal.x, centerBottomGoal.y) | run;
    mb.moveTo(centerBottomGoal.x, centerBottomGoal.y) | run;

    // go towards match3 backwards
    mb.moveTo(48, 48).reverse() | run;

    // turn to and go to match3
    mb.turnTo(70, match3) | run;
    target_point = make_machloader_point({ 67.71_in, match3 }, 7_in);
    mb.moveTo(target_point.x, target_point.y) | run;

    // go to goal
    mb.moveTo(24_in, long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;

    // move towards balls
    drivetrain.moveTank(1.0_volt, -1.0_volt);
    pros::delay(400);

    // balls
    mb.moveTo(-30, 34.5)
        .drive_ErrorTolerance(7_in)
        .drive_ToleranceDuration(0.01_sec)
        .drive_minVolt(0.2_volt) |
      run;

    // matchloader 2
    mb.boomerang(-58_in, match4, 180)
        .lead(0.5)
        .drive_ErrorTolerance(3_in)
        .drive_largeErrorTolerance(4_in)
        .drive_maxVolt(0.4_volt) |
      run;

    // go to goal
    mb.moveTo(-24_in, long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;

    // go to park
    mb.boomerang(-60.755, 18.904, 270).lead(0.3, 0.1) | run;

    // mb.boomerang(-58_in, match1, 180)
    //     .lead(0.65, 0.42)
    //     .lead2DistThreshold(14_in)
    //     .k_lat(0.15, false)
    //     .drive_minVolt(0.2_volt)
    //     .drive_maxVolt(0.5_volt)
    //     .drive_ErrorTolerance(3_in)
    //     .drive_largeErrorTolerance(4_in)
    //     .drive_kd(linear_pid.get_kd() * 0.8) |
    //   run;

    // go to goal
    // mb.moveTo(-23.9_in, 2_tile).reverse().timeout(1_sec) | run;
    // mb.moveTo(-24_in, long_goal - 0.5_in)
    //     .reverse()
    //     .timeout(1.1_sec)
    //     .k_lat(0.0) |
    //   run;
    //
    // intake::set(intake::scoring_long);
    // // pros::delay(3200);
    // intake::set(intake::intake);
    // matchloader::set(inactive);
    //
    // // color sort for red
    // // auto_alliance = alliance_t::red;
    // intake::set(intake::scoring_long);
    //
    // // go through the balls
    // drivetrain.moveTank(-1.0_volt, 1_volt);
    // pros::delay(400);
    //
    // // balls
    // mb.moveTo(30, 34.5)
    //     .drive_ErrorTolerance(7_in)
    //     .drive_ToleranceDuration(0.01_sec)
    //     .drive_minVolt(0.2_volt) |
    //   run;
    //
    // // matchloader 2
    // mb.boomerang(58_in, match2, 0)
    //     .lead(0.55, 0.05)
    //     // .k_lat(0.15, true)
    //     // .k_lat(std::nullopt)
    //     .k_lat(0.05, true)
    //     .drive_ErrorTolerance(3_in)
    //     .drive_largeErrorTolerance(4_in)
    //     .drive_maxVolt(0.5_volt)
    //     .drive_kd(linear_pid.get_kd() * 0.8) |
    //   run;
    //
    // // go to goal
    // mb.moveTo(24_in, long_goal).reverse().timeout(1.0_sec).k_lat(0.0) | run;
    //
    // // LaserResets({ &left_laser_model });
    // intake::set(intake::scoring_long);
    // // pros::delay(3200);
    // intake::set(intake::intake);
    // matchloader::set(inactive);
    //
    // intake::set(intake::scoring_long);
    //
    // // go before matchloader
    //
    // drivetrain.moveTank(1.0_volt, -1.0_volt);
    // pros::delay(200);
    //
    // vexmaps::DistanceSensorConfig new_config = distance_sensor_config;
    //
    // new_config.maxDistanceDifference = 5_in;
    //
    // // change front model to accept larger changes due to drift in wheels
    // front_laser_model.setConfig(new_config);
    //
    // SmootherConfig new_abg_config = smoother_config;
    //
    // // new_abg_config.alpha_x = 0.6;
    // new_abg_config.alpha_y = 0.6;
    //
    // smoother_model.changeConfiguration(new_abg_config);
    //
    // mb.moveTo(44, match3 + 2.0_in).drive_ToleranceDuration(50_msec) | run;
    //
    // front_laser_model.setConfig(distance_sensor_config);
    // smoother_model.changeConfiguration(smoother_config);
    //
    // // LaserResets({ &front_laser_model });
    //
    // matchloader::set(active);
    // intake::set(intake::intake);
    //
    // // go into matchloader
    // // mb.turnTo(58, match3) | run;
    // mb.turnTo(70, match3) | run;
    //
    // // mb.distanceAtHeading(14.7_in).timeout(1_sec) | run;
    // // mb.moveTo(57.7, match3) | run;
    // target_point = make_machloader_point({ 67.71_in, match3 }, 7_in);
    // mb.moveTo(target_point.x, target_point.y) | run;
    //
    // // go to goal
    //
    // mb.moveTo(24_in, -long_goal - 1.0_in)
    //     .reverse()
    //     .timeout(1.2_sec)
    //     .k_lat(0.0) |
    //   run;
    //
    // intake::set(intake::scoring_long);
    // // pros::delay(3200);
    // intake::set(intake::intake);
    // matchloader::set(inactive);
    //
    // // move away from goal
    // // mb.moveTo(40.34, -60.67) | run;
    // drivetrain.moveTank(-1.0_volt, 1.0_volt);
    // pros::delay(400);
    //
    // // move to other side of field
    // mb.moveTo(-24, -34.27)
    //     .drive_ErrorTolerance(7_in)
    //     .drive_ToleranceDuration(10_msec) |
    //   run;
    // // pros::delay(100);
    // // LaserResets({ &front_laser_model, &left_laser_model });
    // // pros::delay(50);
    //
    // // go to right before matchloader
    // mb.moveTo(-46.5, match4).drive_ToleranceDuration(50_msec) | run;
    // matchloader::set(active);
    //
    // // go into matchloader
    // // mb.turnTo(-58, match4) | run;
    // mb.turnTo(-70, match4)
    //     .turn_toleranceDuration(50_msec)
    //     .turn_kd(angular_pid.get_kd() * 0.8) |
    //   run;
    // // LaserResets({ &left_laser_model });
    //
    // // mb.distanceAtHeading(13.5_in).timeout(1_sec) | run;
    // // mb.moveTo(-57.7, match4) | run;
    // target_point = make_machloader_point({ -67.71_in, match4 }, 7_in);
    // mb.moveTo(target_point.x, target_point.y) | run;
    //
    // // go to goal
    // mb.moveTo(-24_in, -long_goal).reverse().timeout(1.0_sec).k_lat(0.0) |
    // run; intake::set(intake::scoring_long);
    // // pros::delay(3200);
    // intake::set(intake::intake);
    // matchloader::set(inactive);
    //
    // // mb.moveTo(-40_in, -2_tile) | run;
    // // mb.turnTo(180) | run;
    //
    // mb.boomerang(-60.755, -18.904, 90)
    //     // .reverse()
    //     // .lead(0.35)
    //     .lead(0.3, 0.1)
    //   // .drive_maxVolt(0.5_volt)
    //   | run;
    //
    // odom_retract::set(active);
    // sideways_tracker.disable();
    // // TODO: disable sideways tracker
    //
    // // mb.moveTo(-62.275, 1.137) | run;
    // mb.moveTo(-62.275, 1.137)
    //     // go max?
    //     .drive_accelSlew(100_volt)
    //     .drive_minVolt(0.3_volt)
    //     .drive_ErrorTolerance(4_in)
    //     .drive_ToleranceDuration(0.01_sec) |
    //   run;
    //
    // // mb.moveTo(-62.275, 1.137).reverse().drive_backwardsAccelSlew(100_volt)
    // |
    // // run;
    //
    // intake::set(intake::intake_disabled);
}

} // namespace sunshine_skills
