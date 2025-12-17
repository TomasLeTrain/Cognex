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
#include "units/Vector2D.hpp"
#include "vexmaps/mcl/distance_model.hpp"
#include <functional>

// do not do anything outside here!

namespace sunshine_skills {

void first_matchloader_mp();
void chained_first_matchloader(Length match1);

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

    auto closeEnough = [&](units::V2Position target,
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

    Length match1 = -normal_match;
    Length match2 = -normal_match;
    Length match3 = normal_match;
    Length match4 = normal_match;

    units::V2Position centerBallOne = { -24_in, 24_in };

    units::V2Position centerTopGoal = { -13_in, 12_in };
    units::V2Position centerBottomGoal = { 13_in, 13_in };

    // move away from park
    mb.moveTo(-37_in, 36_in)
        // .drive_errorTolerance(5_in)
        // .drive_toleranceDuration(50_msec)
        // .drive_chainErrorTolerance(8_in)
        // .setChainTime(50_msec)
		|
      chain;

    // mb.turnTo(centerBallOne.x, centerBallOne.y) | run;
    // mb.moveTo(centerBallOne.x, centerBallOne.y) | run;

    // move to top center goal
    mb.turnTo(centerTopGoal.x, centerTopGoal.y)
        // .turn_errorTolerance(10 * deg)
        // .turn_toleranceDuration(0_msec)
        // .turn_chainErrorTolerance(30 * deg)
        // .setChainTime(0.001_msec)
		|
      chain;

    mb.moveTo(centerTopGoal.x, centerTopGoal.y).k_lat(0.3) | chain;

    chain.waitUntil(closeEnough(centerTopGoal, 4_in));
    intake::score_middle();
    start_time = now();

    // wait for 2 seconds
    while (!timeoutDone(start_time, 2_sec)) {
        pros::delay(10);
    }

    // exit any motions if the are somehow still executing
    chain.exitCurrent();

    // move back
    drivetrain.moveTank(-1.0_volt, -0.9_volt);
    pros::delay(150);
    intake::in();

    // move to center balls 2
    mb.moveTo(-24_in, -24_in).k_lat(0.3)
      // .drive_errorTolerance(7_in)
      // .drive_toleranceDuration(0_sec)
      | chain;

    mb.moveTo(-39.823, -41.915).drive_minVolt(0.5_volt) | chain;
    mb.boomerang(-55.5_in, match1, 180).lead(0.2).closeThreshold(6_in) | chain;

    // wait for center balls to drop the matchloader
    chain.waitUntil(closeEnough({ -24_in, -24_in }, 7_in));

    matchloader::down();

    // wait to get to matchloader
    chain.wait();
    // matchloader 1
    pros::delay(1500);

    // move to goal
    mb.moveTo(-24_in, -long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | async;

    // score goal 1
    async.waitUntil(closeEnough({ -24_in, -long_goal }, 6_in));

    intake::score_long();
    start_time = now();

    // wait for 2 seconds
    while (!timeoutDone(start_time, 2000_msec)) {
        pros::delay(10);
    }

    // exit any motions if the are somehow still executing
    async.exitCurrent();
    matchloader::up();

    // move towards balls
    drivetrain.moveTank(1.0_volt, -1.0_volt);
    pros::delay(200);
    intake::in();
    pros::delay(200);

    // get balls
    mb.moveTo(24, -24).k_lat(0.3)
      // .drive_errorTolerance(7_in)
      // .drive_toleranceDuration(0_sec)
      | chain;

    // get close to matchload 2
    mb.moveTo(48, match2).drive_toleranceDuration(0_msec) | chain;

    // wait to get to balls to pull down matchloader
    chain.waitUntil(closeEnough({ 24_in, -24_in }, 6_in));
    matchloader::down();
    chain.wait();

    // turn to matchload 2
    mb.turnTo(0).turn_toleranceDuration(0_msec) | run;

    // target_point = make_machloader_point({ 67.71_in, match2 }, 11_in);

    // just go straight
    mb.moveTo(60, RobotGetPose().y) | run;
    pros::delay(2000);

    // move to goal
    mb.moveTo(24_in, -long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | async;

    async.waitUntil(closeEnough({ 24_in, -24_in }, 6_in));

    intake::score_long();
    start_time = now();

    // wait for 2 seconds
    while (!timeoutDone(start_time, 2000_msec)) {
        pros::delay(10);
    }
    async.exitCurrent();
    matchloader::up();
    intake::in();

    // simulate going over park

    // mb.moveTo(48_in, 24_in)
    //     .drive_errorTolerance(7_in)
    //     .drive_toleranceDuration(0_sec) |
    //   run;

    // get close to park
    mb.boomerang(60.755, -18.904, 90).lead(0.3, 0.1) | chain;
    // align with the park well
    mb.moveTo(63, -17).drive_maxVolt(0.5_volt).drive_minVolt(0.1_volt) | chain;
    chain.wait();

    // here the thresholds likely need to be bigger for reseting to happen
    // we later reset them back to what they should be
    setMaxDistanceThresholdAll(10_in);

    // go forwards
    drivetrain.moveTank(0.5_volt, 0.5_volt);
    pros::delay(1400);
    // here localization is likely messed up, so we back up a bit to reset on
    // the park

    drivetrain.moveTank(0.2_volt, 0.2_volt);
    pros::delay(400);

    // explode center balls
    matchloader::down();
    mb.moveTo(24, 24) | run;
    matchloader::up();

    // reset the max distance as wel hope we have the right location
    resetMaxDistanceThresholdAll();

    // go to bottom goal
    mb.turnTo(centerBottomGoal.x, centerBottomGoal.y)
        .turn_toleranceDuration(0_sec) |
      chain;
    mb.moveTo(centerBottomGoal.x, centerBottomGoal.y) | chain;

    // wait until robot is close enough
    chain.waitUntil(closeEnough(centerBottomGoal, 5_in));
    intake::score_bottom();
    start_time = now();

    // wait for 2 seconds
    while (!timeoutDone(start_time, 2_sec)) {
        pros::delay(10);
    }

    // exit any motions if the are somehow still executing
    chain.exitCurrent();

    // go towards match3 backwards
    mb.moveTo(48, match3).reverse().drive_toleranceDuration(0_sec) | run;

    // turn to and go to match3
    matchloader::down();
    mb.turnTo(70, match3).turn_toleranceDuration(0.01_sec) | run;
    // target_point = make_machloader_point({ 67.71_in, match3 }, 11_in);
    mb.moveTo(60, match3) | run;
    pros::delay(2000);

    // go to goal
    mb.moveTo(24_in, long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;

    // wait until robot is close enough
    chain.waitUntil(closeEnough({ 24_in, long_goal }, 6_in));
    intake::score_long();
    start_time = now();

    // wait for 2 seconds
    while (!timeoutDone(start_time, 2_sec)) {
        pros::delay(10);
    }

    // exit any motions if the are somehow still executing
    chain.exitCurrent();

    matchloader::up();
    intake::in();

    // move towards balls
    drivetrain.moveTank(1.0_volt, -1.0_volt);
    pros::delay(500);

    // go through first center (does not have any balls)
    mb.moveTo(-30, 34.5)
        .drive_errorTolerance(7_in)
        .drive_toleranceDuration(0.01_sec)
        .drive_minVolt(0.2_volt) |
      run;

    // matchloader 4
    matchloader::down();
    mb.boomerang(-55.8_in, match4, 180)
        .lead(0.5)
        // .drive_errorTolerance(3_in)
        // .drive_largeErrorTolerance(4_in)
        .drive_minVolt(0.1_volt)
        .drive_maxVolt(0.4_volt) |
      chain;
    mb.moveTo(-60_in, match4) | chain;
    pros::delay(1500);

    // go to goal
    mb.moveTo(-24_in, long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;

    // wait until robot is close enough
    chain.waitUntil(closeEnough({ -24_in, long_goal }, 6_in));
    intake::score_long();
    start_time = now();

    // wait for 2 seconds
    while (!timeoutDone(start_time, 2_sec)) {
        pros::delay(10);
    }

    // exit any motions if the are somehow still executing
    chain.exitCurrent();

    matchloader::up();
    intake::in();

    // go to park
    mb.boomerang(-60.755, 18.904, 270).lead(0.3, 0.1) | run;

    // go into park
    drivetrain.moveTank(0.3_volt, 0.3_volt);
    pros::delay(500);
}

void first_matchloader_mp() {
    using namespace blazing::lyfast;
    geometry::Line line({ -23.6_in, -23.6_in }, { -34.72_in, -39.79_in });

    geometry::CubicBezier cubic({ -34.72_in, -39.79_in },
                                // { -36.58_in, -41.79_in },
                                { -38.5_in, -43.8_in },
                                { -36.86_in, -46.7_in },
                                { -56_in, -46.7_in });

    geometry::Spline spline({ &line, &cubic });

    mp::RobotConstraints robot_constraints(10.5_in,
                                           // pretty good
                                           0.08,
                                           //
                                           // 0.043,
                                           // 0.08,
                                           // 0.2,
                                           // 1.0,
                                           3.25_in,
                                           450_rpm,
                                           12_lb,
                                           6.0f);

    // mp::LinearConstraints linear_constraints(20_inps, 20.0_mps2, 2.0_mps2);
    mp::LinearConstraints linear_constraints(40_inps, 20.0_mps2, 2.0_mps2);
    // 3.0_mps2);
    // 1.6_mps2);
    // effectively infinity
    // mp::AngularConstraints angular_constraints(0.8_radps, 0.25_radps2,
    // 0.1_radps2);
    mp::AngularConstraints angular_constraints(20_radps, 20_radps2, 20_radps2);

    mp::Constraints constraints(robot_constraints,
                                linear_constraints,
                                angular_constraints);

    mp::Trajectory spline_trajectory(
      &spline,
      constraints,
      {
        // lyfast::mp::PointConstraint {
        //                              .timeframe = 18_in,
        //                              .vel = 10_inps,
        //                              },
      },
      20_inps,
      0_inps,
      0.1_in);

    drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);

    // run spline on ramsette
    Ramsete(controllers, vexmaps_chassis, &spline_trajectory, 0.7, 35.0)
        .drive_errorTolerance(1_in)
        .drive_largeErrorTolerance(6_in)
        // mainly uses half circle to exit
        .halfcircleTolerance(0_in, 5_in) |
      run;

    pros::delay(100);
    drivetrain.setBrakeMode(pros::v5::MotorBrake::coast);
}

void chained_first_matchloader(Length match1) {
    // boomerang to matchloader 1
    mb.moveTo(-39.823, -41.915).drive_minVolt(0.5_volt) | chain;
    mb.boomerang(-55.5_in, match1, 180).lead(0.2).closeThreshold(6_in)
      // .drive_errorTolerance(3_in)
      // .drive_largeErrorTolerance(4_in)
      // .drive_maxVolt(0.4_volt)
      | chain;
    chain.wait();
}

} // namespace sunshine_skills
