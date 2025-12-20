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

    Length match1 = -normal_match;
    Length match2 = -normal_match;
    Length match3 = normal_match;
    Length match4 = normal_match;

    units::V2Position centerBallOne = { -24_in, 24_in };

    units::V2Position centerTopGoal = { -12.25_in, 10.8_in };
    units::V2Position centerBottomGoal = { 13_in, 13_in };

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(active);

    // make everything be hold
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // don't color sort at the beginning balls
    intake::setColorSortEnabled(false);

    // move away from park
    mb.turnTo(-37_in, 36_in) | chain;
    mb.moveTo(-37_in, 36_in)
      // .drive_errorTolerance(5_in)
      // .drive_toleranceDuration(50_msec)
      // .drive_chainErrorTolerance(8_in)
      // .setChainTime(50_msec)
      | chain;

    // mb.turnTo(centerBallOne.x, centerBallOne.y) | run;
    // mb.moveTo(centerBallOne.x, centerBallOne.y) | run;

    // move to top center goal
    mb.turnTo(centerTopGoal.x, centerTopGoal.y)
      // .turn_errorTolerance(10 * deg)
      // .turn_toleranceDuration(0_msec)
      // .turn_chainErrorTolerance(30 * deg)
      // .setChainTime(0.001_msec)
      | chain;

    mb.moveTo(centerTopGoal.x, centerTopGoal.y).k_lat(0.3) | chain;

    // wait until its close to balls and deploy
    chain.waitUntil(closeEnough({ -27_in, 27_in }, 6.5_in));
    matchloader::down();

    chain.waitUntil(closeEnough(centerTopGoal, 6_in));
    intake::score_middle();
    start_time = now();

    // wait for 2 seconds or until it detects its gonna score a red ball
    while (!timeoutDone(3000_msec, start_time) ||
           intake::getMiddleDetectedColor() == alliance_t::red) {
        pros::delay(10);
    }

    // exit any motions if the are somehow still executing
    chain.exitAll();
    matchloader::up();

    // move back
    drivetrain.moveTank(-1.0_volt, -0.9_volt);

    // outtake for a slight amount of time, as red ball might be in the way of
    // middle intake piston
    intake::out();
    pros::delay(150);
    // stops moving due to the motion thats coming up

    // start intaking to avoid outtaking balls
    intake::in();

    // also enable color to sort out blue balls
    intake::setColorSortEnabled(true);
    auto_alliance = alliance_t::red;

    // move to center balls 2
    mb.moveTo(-27.5_in, -24_in).k_lat(0.3)
      // .drive_errorTolerance(7_in)
      // .drive_toleranceDuration(0_sec)
      | chain;

    // scuff drift controller (causes slight overshoot and correction to the
    // desired point)
    mb.moveTo(-41, -43) | chain;

    // gets close to the target point
    mb.boomerang(-55_in, match1, 180)
        .lead(0.6)
        // .k_lat(0.4, false)
        .closeThreshold(6_in)
        .drive_maxVolt(0.4_volt) |
      chain;
    mb.moveTo(-60_in, match1) | chain;

    // wait for center balls to drop the matchloader
    chain.waitUntil(closeEnough({ -22_in, -14_in }, 6_in));

    matchloader::down();

    // wait to get to matchloader
    chain.wait();

    // matchloading, don't color sort anything
    intake::setColorSortEnabled(false);

    // matchloader 1
    pros::delay(1500);

    // move to goal
    mb.moveTo(-24_in, -long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | async;

    // score goal 1
    async.waitUntil(closeEnough({ -32_in, -long_goal }, 4_in));

    intake::score_long();

    // wait for 2 seconds
    pros::delay(1500);

    // exit any motions if the are somehow still executing
    async.exitAll();
    matchloader::up();

    // move towards balls
    drivetrain.moveTank(1.0_volt, -1.0_volt);
    pros::delay(200);
    intake::in();
    pros::delay(200);

    // get balls
    mb.moveTo(20, -24.5).k_lat(0.3)
      // .drive_errorTolerance(7_in)
      // .drive_toleranceDuration(0_sec)
      | chain;

    // get close to matchload 2

    mb.turnTo(45, match2)
        .turn_chainErrorTolerance(30_stDeg)
        .executeBeforeMotion([] {
            // localization seems to get thrown off here, temporarily give
            // greater greater distance threshold and mcl importance to not lose
            // pose
            setMaxDistanceThresholdAll(10_in);
            setSmootherAlphas(smoother_config.alpha_x * 1.5,
                              smoother_config.alpha_y * 1.5);
        }) |
      chain;
    mb.moveTo(45, match2).drive_toleranceDuration(0_sec).executeAfterMotion([] {
        // reset after motion executed to avoid measuring obstacles like
        // matchloader (although obstacle detection should already be taking
        // care of that )
        resetMaxDistanceThresholdAll();
        resetSmootherConfig();
    })
      // .only_y(true).k_lat(0.0)
      // .drive_kd(linear_pid.get_kd() * 0.5)
      | chain;

    // wait to get to balls to pull down matchloader
    chain.waitUntil(closeEnough({ 18_in, -26_in }, 5_in));
    matchloader::down();

    // spawn as task since its blocking
    pros::Task([] {
        intake::throwOutDetectedBall(std::nullopt, 3_sec);
    });

    chain.wait();

    // turn to matchload 2
    mb.turnTo(0).turn_toleranceDuration(50_msec) | run;

    // target_point = make_machloader_point({ 67.71_in, match2 }, 11_in);

    // just go straight (except it can still go sideways if localization changes
    // itself?)
    mb.moveTo(60, RobotGetPose().y).k_lat(0.0) | run;
    // mb.distanceAtHeading(
    //   RobotGetPose().distanceTo({ 60_in, RobotGetPose().y })) |
    //   run;
    pros::delay(2000);

    // move to goal
    mb.moveTo(24_in, -long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | async;

    async.waitUntil(closeEnough({ 24_in, -24_in }, 8_in));

    intake::score_long();
    // here there should be 3 random blocks, 3 blue and 3 red

    // we wait a bit to score at least 1 of the random blocks
    pros::delay(300);

    // then we just wait for a red one to show up
    intake::waitUntilMiddleColor(alliance_t::red, 1_sec);
    // here we set the intake to outtake for a bit
    intake::out();
    pros::delay(200);
    // then we just score to the top to keep the 3 reds
    intake::set(intake::scoring_long_top_balls);
    pros::delay(700);

    async.exitAll();
    matchloader::up();
    intake::in();

    // get close to park
    mb.boomerang(63.755, -20.904, 90).lead(0.3) | chain;

    // align with the park well
    // mb.moveTo(63, -17).drive_maxVolt(0.5_volt).executeBeforeMotion([] {
    //     horizontal_tracker.setDisabled(true);
    //     odom_retract::retractOdom();
    // }) |
    //   chain;

    chain.wait();

    // disable horizontal odom
    horizontal_tracker.setDisabled(true);
    odom_retract::retractOdom();

    // here the thresholds likely need to be bigger for reseting to happen
    // we later reset them back to what they should be
    setMaxDistanceThresholdAll(10_in);

    drivetrain.moveTank(0.2_volt, 0.125_volt);
    // move closer to the park slowly, also gives time for accurate start roll
    pros::delay(400);

    // roll is hopefully accurate here
    double start_roll = imu.get_roll();
    std::cout << "start roll: " << start_roll << std::endl;
    pros::delay(400);

    // get over first part of park
    drivetrain.moveTank(0.5_volt, 0.4_volt);

    // positive only when going up at the park
    while (std::abs(imu.get_roll() - start_roll) < 4) {
        std::cout << imu.get_roll() - start_roll << std::endl;
        controller.rumble(".");
        pros::delay(20);
    }
    std::cout << "done" << std::endl;
    pros::delay(100);
    // hopefully got some initial balls

    // go fast again to avoid getting stuck
    drivetrain.moveTank(0.3_volt, 0.25_volt);
    // go through rest of the balls
    pros::delay(300);
    // now in park, go slowish
    drivetrain.moveTank(0.2_volt, 0.15_volt);
    pros::delay(400);
    matchloader::down();
    pros::delay(400);

    // get out of park
    drivetrain.moveTank(0.5_volt, 0.4_volt);

    // when all but one wheel are over park
    // negative while going down in the motion
    while (std::abs(imu.get_roll() - start_roll) < 8) {
        std::cout << imu.get_roll() - start_roll << std::endl;
        controller.rumble(".");
        pros::delay(20);
    }
    std::cout << "done" << std::endl;

    // give time to finally get out of park
    pros::delay(400);
    // over park now, can continue with run

    // enable odom again
    horizontal_tracker.setDisabled(false);
    odom_retract::lowerOdom();

    // the threshold for distance is still pretty big as localization has to be
    // pretty good
    //
    // explode center balls
    matchloader::down();
    mb.moveTo(24, 24).drive_maxVolt(0.5_volt) | run;
    matchloader::up();

    // reset the max distance as wel hope we have the right location
    resetMaxDistanceThresholdAll();

    // go to bottom goal
    mb.turnTo(centerBottomGoal.x, centerBottomGoal.y) | chain;
    mb.moveTo(centerBottomGoal.x, centerBottomGoal.y) | chain;

    // wait until robot is close enough
    chain.waitUntil(closeEnough(centerBottomGoal, 5_in));
    intake::score_bottom();
    matchloader::down();

    pros::delay(3000);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    // go towards match3 backwards
    mb.moveTo(48, match3).reverse() | run;

    // turn to and go to match3
    matchloader::down();
    intake::in();

    mb.turnTo(70, match3) | run;
    // target_point = make_machloader_point({ 67.71_in, match3 }, 11_in);
    mb.moveTo(60, match3) | run;
    pros::delay(2000);

    // go to goal
    mb.moveTo(24_in, long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;

    // wait until robot is close enough
    chain.waitUntil(closeEnough({ 32_in, long_goal }, 4_in));
    intake::score_long();

    // wait for 2 seconds
    pros::delay(1500);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    matchloader::up();
    intake::in();

    // move towards balls
    // drivetrain.moveTank(1.0_volt, -1.0_volt);
    // pros::delay(500);

    // go through first center (does not have any balls)
    mb.moveTo(-28, 35.5)
        .drive_errorTolerance(7_in)
        .drive_toleranceDuration(0.01_sec)
        .drive_minVolt(0.2_volt) |
      chain;

    // matchloader 4
    matchloader::down();
    mb.boomerang(-60_in, match4, 180)
        .lead(0.3)
        // .drive_errorTolerance(3_in)
        // .drive_largeErrorTolerance(4_in)
        .closeThreshold(7_in)
        .drive_minVolt(0.1_volt)
        .drive_maxVolt(0.4_volt) |
      chain;
    chain.wait();
    pros::delay(2000);
    // mb.moveTo(-60_in, match4) | chain;

    // go to goal
    mb.moveTo(-24_in, long_goal).reverse().timeout(1.1_sec).k_lat(0.0) | run;

    // wait until robot is close enough
    chain.waitUntil(closeEnough({ -32_in, long_goal }, 4_in));
    intake::score_long();

    // wait for 2 seconds
    pros::delay(1500);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    matchloader::up();
    intake::in();

    // go to park
    mb.boomerang(-60.755, 18.904, 270)
        .lead(0.3, 0.1)
        // exit immediately to go into park
        .drive_minVolt(0.1_volt)
        .drive_toleranceDuration(0_msec)
        .drive_errorTolerance(5_in) |
      run;

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

// void first_matchloader_mp() {
//     using namespace blazing::lyfast;
//     geometry::Line line({ -23.6_in, -23.6_in }, { -34.72_in, -39.79_in });
//
//     geometry::CubicBezier cubic({ -34.72_in, -39.79_in },
//                                 // { -36.58_in, -41.79_in },
//                                 { -38.5_in, -43.8_in },
//                                 { -36.86_in, -46.7_in },
//                                 { -56_in, -46.7_in });
//
//     geometry::Spline spline({ &line, &cubic });
//
//     mp::RobotConstraints robot_constraints(10.5_in,
//                                            // pretty good
//                                            0.08,
//                                            //
//                                            // 0.043,
//                                            // 0.08,
//                                            // 0.2,
//                                            // 1.0,
//                                            3.25_in,
//                                            450_rpm,
//                                            12_lb,
//                                            6.0f);
//
//     // mp::LinearConstraints
//     linear_constraints(20_inps, 20.0_mps2, 2.0_mps2); mp::LinearConstraints
//     linear_constraints(40_inps, 20.0_mps2, 2.0_mps2);
//     // 3.0_mps2);
//     // 1.6_mps2);
//     // effectively infinity
//     // mp::AngularConstraints angular_constraints(0.8_radps, 0.25_radps2,
//     // 0.1_radps2);
//     mp::AngularConstraints angular_constraints(20_radps, 20_radps2,
//     20_radps2);
//
//     mp::Constraints constraints(robot_constraints,
//                                 linear_constraints,
//                                 angular_constraints);
//
//     mp::Trajectory spline_trajectory(
//       &spline,
//       constraints,
//       {
//         // lyfast::mp::PointConstraint {
//         //                              .timeframe = 18_in,
//         //                              .vel = 10_inps,
//         //                              },
//       },
//       20_inps,
//       0_inps,
//       0.1_in);
//
//     drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);
//
//     // run spline on ramsette
//     Ramsete(controllers, vexmaps_chassis, &spline_trajectory, 0.7, 35.0)
//         .drive_errorTolerance(1_in)
//         .drive_largeErrorTolerance(6_in)
//         // mainly uses half circle to exit
//         .halfcircleTolerance(0_in, 5_in) |
//       run;
//
//     pros::delay(100);
//     drivetrain.setBrakeMode(pros::v5::MotorBrake::coast);
// }
//
// void chained_first_matchloader(Length match1) {
//     // boomerang to matchloader 1
//     mb.moveTo(-39.823, -41.915).drive_minVolt(0.5_volt) | chain;
//     mb.boomerang(-55.5_in, match1, 180).lead(0.2).closeThreshold(6_in)
//       // .drive_errorTolerance(3_in)
//       // .drive_largeErrorTolerance(4_in)
//       // .drive_maxVolt(0.4_volt)
//       | chain;
//     chain.wait();
// }

} // namespace sunshine_skills
