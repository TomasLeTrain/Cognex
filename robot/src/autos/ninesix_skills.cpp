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

namespace ninesix_skills {

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

    // auto closeEnough = [](units::V2Position target,
    //                       Length threshold) -> std::function<bool()> {
    //     return [target, threshold] -> bool {
    //         return RobotGetPose().distanceTo(target) < threshold;
    //     };
    // };

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

    units::V2Position centerTopGoalFirst = { -8.0_in, 7.4_in };
    units::V2Position centerTopGoalSecond = { 1_in, 0_in };
    units::V2Position centerBottomGoalFirst = { 12_in, 12_in };
    units::V2Position centerBottomGoalSecond = { -12_in, -11_in };

    intake::setSkillsMiddleScoring(true);

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(inactive);

    RobotSetPose(-47.2, 14.9, 0);
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // only intake bottom balls to save time
    // intake::set(intake::intake_bottom_balls);
    intake::setColorSortEnabled(false);
    // intake::setColorSortEnabled(true);
    // auto_alliance = alliance_t::red;
    // intake::in();
    pros::delay(10);
    intake::set(intake::intake_bottom_balls);

    // mb.moveTo(-25, 23.4).drive_maxVolt(0.7_volt).timeout(0.4_sec) | run;
    //
    mb.moveTo(-31.5, 19.4)
      // .drive_maxVolt(0.3_volt)
      | run;
    // pros::delay(200);
    // intake::set(intake::intake_disabled);
    // mb.moveTo(-25.7, 23.4) | run;

    // color sort balls here
    // chain.waitUntil(closeEnough({ -25_in, 23.4_in }, 10_in));
    // matchloader::down();
    // pros::delay(300);

    pf_model.setDisabled(true);
    mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
      // .direction(AngularDirection::LEFT)
      // .executeAfterMotion([] {
      //     // intake::set(intake::intake_disabled_open_middle);
      // })
      | run;
    pf_model.setDisabled(false);

    mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
        .k_lat(0.3)
        .drive_maxVolt(0.45_volt)
        .executeBeforeMotion([] {
            pros::Task([] {
                matchloader::down();
                pros::delay(240);
                matchloader::up();
            });
        }) |
      chain;

    chain.waitUntil(closeEnough({ -8_in, 8_in }, 5.5_in));
    // controller.rumble(".");
    // outtake slightly in case first ball is stuck
    // intake::out();
    // pros::delay(50);
    // intake::set(intake::scoring_middle_bottom_balls);
    intake::out();
    pros::delay(150);
    intake::set(intake::scoring_middle_bottom_balls);
    chain.exitAll();
    drivetrain.moveTank(0.1_volt, 0.2_volt);
    // start_time = now();
    pros::delay(1400);
    // pros::delay(5000);

    // wait for 3 seconds or until it detects its gonna score a blue ball
    // while (!timeoutDone(3000_msec, start_time) ||
    //        intake::getMiddleDetectedColor() == alliance_t::blue) {
    //     pros::delay(10);
    // }

    // exit any motions if the are somehow still executing
    // matchloader::up();

    // move back

    mb.moveTo(-48, match1 - 0.7_in)
        .reverse()
        .only_y(true)
        .drive_backwardsAccelSlew(0.1_volt)
        .executeAfterMotion([] {
            intake::in();
            intake::setColorSortEnabled(false);
            matchloader::down();
        }) |
      run;

    mb.turnTo(-70, match1) | chain;
    mb.moveTo(-58.5, match1).drive_maxVolt(0.5_volt) | chain;

    chain.waitUntil([] -> bool {
        return RobotGetPose().x < -53_in;
    });
    pros::delay(1600);
    chain.exitAll();

    // mb.arc(330, -1.0).direction(AngularDirection::RIGHT).reverse() | run;

    mb.moveTo(-25, 61.1)
        .reverse()
        .executeAfterMotion([] {
            intake::set(intake::intake_disabled);
            matchloader::up();
        })
        .drive_chainErrorTolerance(8_in)
        .setChainTime(0_sec)
        .drive_minVolt(0.5_volt) |
      chain;
    // go to other side
    mb.moveTo(25, 59)
        .reverse()
        .drive_chainErrorTolerance(7_in)
        .setChainTime(30_msec)
        .drive_minVolt(0.2_volt) |
      chain;
    // get on same y
    mb.moveTo(42, long_goal + 0.2_in).reverse().only_y(true) | chain;

    // turn to goal, reversed
    mb.turnTo(10_in, long_goal).reverse() | chain;
    // chain.wait();

    // LaserResets({ &left_laser_model });

    // go to goal, reversed
    // mb.moveTo(24_in, long_goal + 0.0_in)
    //     .reverse()
    //     .timeout(1.4_sec)
    //     .k_lat(0.0)
    //     .closeThreshold(8_in)
    //     .executeAfterMotion([] {
    //         pros::delay(100);
    //         drivetrain.moveTank(-0.0_volt, -0.15_volt);
    //     })
    //   // .only_x(true)
    //   | async;

    mb.moveTo(25_in, long_goal + 0.0_in).reverse()
      // .timeout(1.4_sec)
      // .k_lat(0.0)
      // .closeThreshold(8_in)
      // .executeAfterMotion([] {
      //     pros::delay(100);
      //     drivetrain.moveTank(-0.0_volt, -0.15_volt);
      // })
      // .only_x(true)
      | chain;

    chain.waitUntil(closeEnough({ 31_in, long_goal }, 3_in));

    intake::score_long();

    // wait for 2 seconds
    pros::delay(500);
    matchloader::down();
    pros::delay(1900);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    // mb.moveTo(55, match2).drive_maxVolt(0.5_volt).executeAfterMotion([] {
    //     intake::in();
    // }) |
    //   run;
    mb.turnTo(46, match2 - 0.0_in) | run;
    // mb.distanceAtHeading(31_in, RobotGetPose().angleTo({ 63_in, match2 }))
    //     .drive_maxVolt(0.4_volt) |
    //   chain;
    mb.moveTo(58, match2 - 0.0_in)
        .drive_maxVolt(0.4_volt)
        .k_lat(0.4)
        .closeThreshold(4_in) |
      chain;

    // let it score last one
    pros::delay(300);
    // outake a bit while going, in case something is stuck
    intake::out();
    pros::delay(200);
    intake::in();

    chain.wait();
    // drivetrain.moveTank(0.1_volt, 0.1_volt);
    // pros::delay(300);
    // drivetrain.moveTank(0.05_volt, 0.05_volt);
    // pros::delay(300);
    // drivetrain.moveTank(0.0_volt, 0.0_volt);
    pros::delay(1200);
    // LaserResets({ &left_laser_model });

    // go to goal to score again
    // mb.turnTo(25_in, long_goal + 0.0_in).reverse() | chain;
    mb.moveTo(25_in, long_goal + 0.0_in).reverse()
      // .timeout(1.4_sec)
      // .k_lat(0.3)
      // changed today!
      // .turn_kp(angular_pid.get_kp() * 0.5)
      // .turn_kd(angular_pid.get_kd() * 0.5)
      // .drive_backwardsAccelSlew(0.4_volt)
      //

      // .drive_backwardsAccelSlew(0.4_volt)
      // .drive_maxVolt(0.7_volt)
      //
      // .closeThreshold(8_in)
      // .executeAfterMotion([] {
      //     pros::delay(100);
      //     drivetrain.moveTank(-0.15_volt, -0.3_volt);
      // })
      // .only_x(true)
      | chain;

    chain.waitUntil(closeEnough({ 32_in, long_goal }, 4_in));

    intake::score_long();

    pros::delay(500);
    matchloader::up();
    pros::delay(1900);

    // exit any motions if the are somehow still executing
    chain.exitAll();

    mb.boomerang(62, 19, 270).lead(0.3).drive_maxVolt(0.5_volt) | chain;

    // let it score last one
    intake::score_long();

    chain.wait();
    // now going for park
    intake::in();

    // disable horizontal odom
    horizontal_tracker.setDisabled(true);
    vertical_tracker.setDisabled(true);
    odom_retract::retractOdom();

    // here the thresholds likely need to be bigger for reseting to happen
    // we later reset them back to what they should be
    setMaxDistanceThresholdAll(10_in);
    setSmootherAlphas(std::nullopt, smoother_config.alpha_y * 2.0);

    drivetrain.moveTank(0.2_volt, 0.25_volt);
    // move closer to the park slowly, also gives time for accurate start roll
    // pros::delay(200);

    // roll is hopefully accurate here
    // double start_roll = imu.get_roll();
    // std::cout << "start roll: " << start_roll << std::endl;
    pros::delay(400);

    // get over first part of park
    //
    drivetrain.moveTank(0.53_volt, 0.63_volt);
    pros::delay(300);
    drivetrain.moveTank(0.4_volt, 0.5_volt);
    pros::delay(1200);
    drivetrain.moveTank(0.2_volt, 0.3_volt);
    pros::delay(800);
    // pull down matchloader only at the pure end
    matchloader::down();
    // pros::delay(300);
    drivetrain.moveTank(0_volt, 0_volt);

    // enable odom again
    horizontal_tracker.setDisabled(false);
    vertical_tracker.setDisabled(false);
    odom_retract::lowerOdom();
    pros::delay(200);

    // the threshold for distance is still pretty big as localization has to be
    // pretty good
    //
    // explode center balls
    // matchloader::down();
    drivetrain.moveTank(-0.4_volt, -0.4_volt);
    pros::delay(100);
    mb.turnTo(180) | run;
    matchloader::up();
    // reset position to guarantee its not wrong at all
    LaserResets({ &back_laser_model, &left_laser_model });
    // LaserResets({ &back_laser_model });

    mb.moveTo(31.5, -18.5).drive_maxVolt(0.5_volt) | run;
    // intake::set(intake::intake_disabled);
    mb.moveTo(23, -23.4).drive_maxVolt(0.3_volt) | run;
    matchloader::up();

    // reset the max distance as we hope we have the right location
    resetMaxDistanceThresholdAll();
    resetSmootherConfig();

    // go to bottom goal

    back_laser_model.disable();

    mb.turnTo(centerTopGoalSecond.x, centerTopGoalSecond.y) | run;
    right_laser_model.disable();
    mb.moveTo(7.3, -6.6)
        .drive_maxVolt(0.3_volt)
        .closeThreshold(6_in)
        .executeBeforeMotion([] {
            pros::Task([] {
                matchloader::down();
                pros::delay(500);
                matchloader::up();
            });
        })
        .executeAfterMotion([] {
            drivetrain.moveTank(0.05_volt, 0.05_volt);
        }) |
      async;

    // mb.distanceAtHeading(20_in).timeout(1.1_sec) | run;
    // pros::delay(10);

    // mb.moveTo(centerTopGoalSecond.x, centerTopGoalSecond.y).k_lat(0.0) | run;

    // chain.waitUntil(closeEnough({ 9_in, 9_in }, 3_in));

    // intake::set(intake::scoring_middle);

    // intake::out();
    // pros::delay(150);

    async.waitUntil(closeEnough({ 8_in, -8_in }, 6_in));

    intake::set(intake::scoring_middle_bottom_balls);
    pros::delay(1900);
    intake::set(intake::scoring_middle_top_balls_skills);
    // pros::delay(10000);
    // pros::delay(4000);

    start_time = now();
    while (!timeoutDone(2100_msec, start_time) &&
           intake::getMiddleDetectedColor() != alliance_t::blue) {
        pros::delay(10);
    }

    // exit any motions if the are somehow still executing
    async.exitAll();

    setSmootherAlphas(smoother_config.alpha_x * 2.0,
                      smoother_config.alpha_y * 2.0);
    // setMaxDistanceThresholdAll(10_in);
    right_laser_model.enable();
    back_laser_model.disable();

    mb.moveTo(48, match3 - 0.4_in)
        .reverse()
        // .drive_maxVolt(0.4_volt)
        .drive_backwardsAccelSlew(0.05_volt)
      // .executeBeforeMotion([] {
      //     matchloader::down();
      // })
      | async;
    pros::delay(400);
    intake::out();
    // pros::delay(100);
    // intake::in();
    async.wait();

    // return;

    resetSmootherConfig();
    // resetMaxDistanceThresholdAll();

    back_laser_model.enable();

    matchloader::down();
    intake::in();

    // mb.turnTo(60, match3 + 0.0_in) | chain;
    mb.turnTo(65, match3 + 0.0_in) | chain;
    mb.moveTo(60.25, match3 + 0.0_in).drive_maxVolt(0.5_volt) | chain;
    // mb.distanceAtHeading(12_in)
    // mb.distanceAtHeading(13_in).drive_maxVolt(0.5_volt) | chain;
    chain.wait();
    pros::delay(1000);

    mb.moveTo(25, -60)
        .reverse()
        .executeAfterMotion([] {
            intake::set(intake::intake_disabled);
            matchloader::up();
        })
        .drive_chainErrorTolerance(8_in)
        .setChainTime(0_sec)
        .drive_minVolt(0.5_volt) |
      chain;
    // go to other side
    mb.moveTo(-25, -59)
        .reverse()
        .drive_chainErrorTolerance(7_in)
        .setChainTime(30_msec)
        .drive_minVolt(0.2_volt)
        .executeBeforeMotion([] {
            // setMaxDistanceThresholdAll(10_in);
            setSmootherAlphas(smoother_config.alpha_x * 2.0,
                              smoother_config.alpha_y * 2.0);
        }) |
      chain;

    // get on same y
    mb.moveTo(-46, -long_goal - 1.3_in).reverse()
      // .only_y(true)
      // .executeBeforeMotion([] {
      //       // LaserResets({ &right_laser_model });
      //   })
      | chain;

    // turn to goal, reversed
    mb.turnTo(0_in, -long_goal).reverse().executeAfterMotion([] {
        resetSmootherConfig();
        resetMaxDistanceThresholdAll();
    }) |
      chain;
    // chain.wait();

    // go to goal, reversed
    mb.moveTo(-25_in, -long_goal).reverse()
      // .timeout(1.3_sec)
      // .k_lat(0.0)
      // .drive_maxVolt(0.8_volt)
      //
      // .closeThreshold(8_in)
      // .executeAfterMotion([] {
      //     drivetrain.moveTank(-0.3_volt, -0.1_volt);
      // })
      // .only_x(true)
      | chain;
    // wait until its close to goal
    chain.waitUntil(closeEnough({ -32_in, -long_goal }, 4_in));

    // drivetrain.moveTank(-0.1_volt, -0.1_volt);
    intake::score_long();

    // wait for 2 seconds
    pros::delay(500);
    matchloader::down();
    pros::delay(2000);

    // exit any motions if they are somehow still executing
    chain.exitAll();

    // drivetrain.moveTank(0.0_volt, 0.0_volt);
    RobotSetPose({ -28.7_in, RobotGetPose().y, RobotGetPose().orientation });
    // LaserResets({ &left_laser_model });

    mb.turnTo(-58, match4 + 0_in) | run;
    mb.moveTo(-58, match4 + 0_in).drive_maxVolt(0.5_volt) | chain;
    // mb.distanceAtHeading(32_in, RobotGetPose().angleTo({ -80_in, match4 }))
    //   .drive_maxVolt(0.5_volt) |
    // chain;

    pros::delay(500);
    intake::in();
    chain.wait();
    pros::delay(1100);

    // LaserResets({ &left_laser_model });

    // mb.turnTo(0_in, current).reverse() | chain;
    // mb.turnTo(0_stDeg).reverse() | chain;
    mb.moveTo(-25_in, -long_goal - 0.0_in).reverse()
      // .timeout(1.4_sec)
      // .k_lat(0.0)

      // changed today!
      // .turn_kp(angular_pid.get_kp() * 0.5)
      // .turn_kd(angular_pid.get_kd() * 0.5)
      // .drive_backwardsAccelSlew(0.4_volt)
      //

      // .closeThreshold(6_in)
      // .executeAfterMotion([] {
      //     mb.arc(180, -0.8)
      //         .timeout(100_sec)
      //         .turn_errorTolerance(0_stDeg)
      //         .turn_largeErrorTolerance(0_stDeg)
      //         .turn_toleranceDuration(100_sec)
      //         .turn_largeToleranceDuration(100_sec) |
      //       async;
      //     // drivetrain.moveTank(-0.3_volt, -0.0_volt);
      // })
      // .only_x(true)
      // .drive_maxVolt(0.4_volt)
      | chain;
    chain.waitUntil(closeEnough({ -32_in, -long_goal }, 4_in));

    intake::score_long();

    pros::delay(1000);
    matchloader::up();
    pros::delay(1200);

    chain.exitAll();

    // make sure new motion doesn't run
    pros::delay(40);
    async.exitAll();

    mb.boomerang(-62, -18, 90)
        .lead(0.3)
        .drive_maxVolt(0.5_volt)
        .drive_minVolt(0.3_volt)
        .drive_largeErrorTolerance(5_in)
        // .halfcircleTolerance(10_in)
        .drive_toleranceDuration(0_sec) |
      run;

    horizontal_tracker.setDisabled(true);
    odom_retract::retractOdom();
    pros::delay(300);

    intake::in();
    //
    drivetrain.moveTank(0.6_volt, 0.7_volt);
    pros::delay(600);
    drivetrain.moveTank(0.4_volt, 0.5_volt);
    pros::delay(200);

    // stop the robot
    drivetrain.moveTank(0.0_volt, 0.0_volt);
}

} // namespace ninesix_skills
