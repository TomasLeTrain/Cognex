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
        .drive_chainErrorTolerance(8_in)
        .setChainTime(50_msec) |
      chain;

    // mb.turnTo(centerBallOne.x, centerBallOne.y) | run;
    // mb.moveTo(centerBallOne.x, centerBallOne.y) | run;

    // move to top center goal
    mb.turnTo(centerTopGoal.x, centerTopGoal.y)
        // .turn_errorTolerance(10 * deg)
        // .turn_toleranceDuration(0_msec)
        .turn_chainErrorTolerance(30 * deg)
        .setChainTime(0.001_msec) |
      chain;

    mb.moveTo(centerTopGoal.x, centerTopGoal.y).k_lat(0.3) | chain;

    chain.wait();
    intake::score_middle();

    // move back
    drivetrain.moveTank(-1.0_volt, -0.9_volt);
    pros::delay(150);
    intake::in();

    // move to center balls 2
    mb.moveTo(-24_in, -24_in)
        .k_lat(0.3)
        .drive_errorTolerance(7_in)
        .drive_toleranceDuration(0_sec) |
      run;

    // TODO: might have to do earlier based on distance?
    matchloader::down();

    // boomerang to matchloader 1
    // mb.boomerang(-58.5_in, match1, 180)
    //     .drive_errorTolerance(3_in)
    //     .drive_largeErrorTolerance(4_in)
    //     .drive_maxVolt(0.4_volt) |
    //   run;
    // first_matchloader_mp();
    chained_first_matchloader(match1);

    // move to goal
    mb.moveTo(-24_in, -long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;
    // TODO: maybe should be based on when its close enough to save score time?
    intake::score_long();

    // TODO: add delay for however long

    // move towards balls
    drivetrain.moveTank(1.0_volt, -1.0_volt);
    pros::delay(400);
    intake::in();

    // TODO: matchload??

    // get balls
    mb.moveTo(24, -24)
        .k_lat(0.3)
        .drive_errorTolerance(7_in)
        .drive_toleranceDuration(0_sec) |
      run;

    // get close to matchload 2
    mb.moveTo(48, match2).drive_toleranceDuration(0_msec) | run;

    matchloader::down();

    // turn to matchload 2
    mb.turnTo(70, match2).turn_toleranceDuration(0_msec) | run;

    target_point = make_machloader_point({ 67.71_in, match2 }, 11_in);
    mb.moveTo(target_point.x, target_point.y) | run;
    // TODO: delay however long

    // move to goal
    mb.moveTo(24_in, -long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;
    // TODO: start when close enough
    intake::score_long();
    // TODO: delay however long
    matchloader::up();
    intake::in();

    // simulate going over park
    mb.moveTo(48_in, 24_in)
        .drive_errorTolerance(7_in)
        .drive_toleranceDuration(0_sec) |
      run;

    // explode center balls
    matchloader::down();
    mb.moveTo(24, 24) | run;

    // go to bottom goal
    mb.turnTo(centerBottomGoal.x, centerBottomGoal.y)
        .turn_toleranceDuration(0_sec) |
      run;
    mb.moveTo(centerBottomGoal.x, centerBottomGoal.y) | run;
    intake::score_bottom();
    // TODO: delay

    // go towards match3 backwards
    mb.moveTo(48, match3).reverse().drive_toleranceDuration(0_sec) | run;

    // turn to and go to match3
    matchloader::down();
    mb.turnTo(70, match3).turn_toleranceDuration(0.01_sec) | run;
    target_point = make_machloader_point({ 67.71_in, match3 }, 11_in);
    mb.moveTo(target_point.x, target_point.y) | run;

    // go to goal
    mb.moveTo(24_in, long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;
    // TODO: start when close enough
    intake::score_long();
    // TODO: delay
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
      run;
    // TODO: delay

    // go to goal
    mb.moveTo(-24_in, long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;
    // TODO: start just at right time
    intake::score_long();
    // TODO: delay
    matchloader::up();
    intake::in();

    // go to park
    mb.boomerang(-60.755, 18.904, 270).lead(0.3, 0.1) | run;
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
