#include "apis.h"
//
#include "auton_globals.h"
#include "autos.h"
#include "blazing/utils.hpp"
#include "globals/blazing_globals.h"
#include "globals/config.h"
#include "globals/device_globals.h"
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
    intake::setColorSortEnabled(false);

    auto make_matchloader_point = [](double sign_x,
                                     double sign_y) -> units::V2Position {
        Length normal_match = 46.7_in;
        return { 67.4_in * sign_x, normal_match * sign_y };
    };

    auto matchload = [make_matchloader_point](double sign_x,
                                              double sign_y,
                                              Time matchload_time) {
        auto make_machloader_pose = [&](units::V2FPosition target,
                                        Length distance) -> units::Pose {
            // auto target_angle = target.angleTo(RobotGetPose());
            auto final_point =
              target + distance * (RobotGetPose() - target).normalize();

            return units::Pose { final_point, final_point.angleTo(target) };
        };

        Length normal_match = 46.7_in;
        units::V2Position target_Point = make_matchloader_point(sign_x, sign_y);
        Length target_dist = 7_in;

        auto func = [&] -> units::Pose {
            return make_machloader_pose(target_Point, target_dist);
        };

        // pull matchloader down regardless
        matchloader::down();

        mb.boomerang(func)
            .timeout(3_sec)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_maxVolt(0.6_volt)
            .k_lat(1.2)
            .lead(0.9) |
          async;

        Length slow_dist = 24_in;
        async.waitUntil([&] -> bool {
            return RobotGetPose().distanceTo(target_Point) < slow_dist;
        });

        async.exitAll();

        mb.boomerang(func)
            .timeout(3_sec)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_maxVolt(0.25_volt)
            .drive_velocityTolerance(03_inps)
            .drive_errorTolerance(10_in)
            .drive_toleranceDuration(0_sec)
            .k_lat(1.2)
            .lead(0.9)
            .executeAfterMotion([] {
                pros::delay(10);
                drivetrain.moveTank(0.25_volt, 0.25_volt);
            }) |
          async;

        async.waitUntil([] -> bool {
            return units::abs(model_manager.getLocalVelocityVector().x) <
                   1_inps;
        });
        // here the robot is close to still, start matchloading

        pros::delay(to_msec(matchload_time));

        async.exitAll();
    };

    auto score_long_goal = [](double sign_x,
                              double sign_y,
                              Time score_time,
                              bool from_matchloader = false) {
        // turn to goal, reversed
        // mb.turnTo(25_in * sign_x, long_goal * sign_y).reverse() |
        // chain; mb.moveTo(25_in * sign_x, long_goal * sign_y)
        //     .reverse()
        //     .k_lat(0.0) |
        //   chain;

        Length long_goal = 47.1_in;
        Length normal_match = 46.7_in;

        units::Pose target_pose = { 26_in * sign_x,
                                    long_goal * sign_y,
                                    sign_x == -1 ? 0_stDeg : 180_stDeg };

        units::Pose other_target_pose = { 24_in * sign_x,
                                          long_goal * sign_y,
                                          sign_x == -1 ? 0_stDeg : 180_stDeg };

        if (from_matchloader)
            mb.arc(target_pose, -1.3)
                .reverse()
                // .drive_chainErrorTolerance()
                .setChainTime(0_sec)
                .drive_minVolt(0.2_volt) |
              chain;
        else
            mb.turnTo(target_pose).reverse().setChainTime(0_sec) | chain;

        mb.boomerang(target_pose).reverse() | chain;

        Length slow_dist = 12_in;
        Length score_dist = 7.5_in;

        chain.waitUntil([&] -> bool {
            return RobotGetPose().distanceTo(target_pose) < slow_dist;
        });

        // exit current boomerang
        chain.exitAll();

        // go into new which is slower
        mb.boomerang(target_pose)
            .drive_maxVolt(0.5_volt)
            .reverse()

            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            // .drive_maxVolt(0.25_volt)
            .drive_velocityTolerance(03_inps)
            .drive_errorTolerance(6_in)
            .drive_toleranceDuration(0_sec)
            .setChainTime(0_sec)

          // .closeThreshold(100_in)
          // .timeout(5_sec)
          // .drive_toleranceDuration(100_sec)
          // .drive_largeToleranceDuration(100_sec)
          // .turn_kp(turn_drive_pid.get_kp() * 2)
          // .turn_kd(turn_drive_pid.get_kd() * 0.5)
          | chain;

        mb.boomerang(other_target_pose)
            // .drive_maxVolt(0.3_volt)
            .reverse()
            .closeThreshold(100_in)
            .timeout(5_sec)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_ki(0)
            .turn_kp(turn_drive_pid.get_kp() * 1.0)
            .turn_kd(turn_drive_pid.get_kd() * 0.8) |
          chain;

        chain.waitUntil(closeEnough(target_pose, score_dist));
        intake::score_long();
        pros::delay(to_msec(score_time));
        chain.exitAll();
        // intake::set(intake::intake_disabled);
        // drivetrain.moveTank(0_volt, 0_volt);
    };

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(inactive);

    RobotSetPose(-45.7, 15, 0);
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // only intake bottom balls to save time
    intake::setColorSortEnabled(false);
    intake::set(intake::intake_bottom_top_backwards);

    mb.moveTo(-31.0, 19.4) | run;

    mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | run;

    mb.boomerang(centerTopGoalFirst.x, centerTopGoalFirst.y, 315_stDeg)
        .k_lat(0.28)
        .lead(0.14)
        .closeThreshold(7_in)
        .drive_maxVolt(0.28_volt)
        .executeBeforeMotion([] {
            pros::Task([] {
                matchloader::down();
                pros::delay(450);
                matchloader::up();
            });
        }) |
      chain;

    chain.waitUntil(closeEnough({ -8_in, 8_in }, 5.5_in));
    intake::set(intake::outtake_open_middle);
    pros::delay(150);
    intake::set(intake::scoring_middle_bottom_balls);
    pros::delay(100);
    intake::set(intake::scoring_middle_bottom_balls_slow);
    chain.exitAll();
    drivetrain.moveTank(0.07_volt, 0.07_volt);

    pros::delay(1600);

    mb.moveTo(-48, match1 - 0.0_in)
        .reverse()
        .only_y(true)
        .drive_maxVolt(0.7_volt)
        // .drive_toleranceDuration(20_msec)
        .executeBeforeMotion([] {
            pros::Task([] {
                // allow last balls to score
                pros::delay(200);
                intake::out();
            });
        })
        .executeAfterMotion([] {
            intake::in();
            matchloader::down();
        }) |
      run;

    mb.turnTo(make_matchloader_point(-1, 1)) | run;
    matchload(-1, 1, 2.5_sec);

    mb.moveTo(-27, 59.5)
        .reverse()
        .executeAfterMotion([] {
            intake::set(intake::intake_disabled);
            matchloader::up();
        })
        .drive_chainErrorTolerance(8_in)
        .setChainTime(0_sec)
        .drive_maxVolt(0.5_volt) |
      chain;

    // mb.boomerang(-27, 60, 180)
    //     .lead(0.2)
    //     .reverse()
    //     .executeAfterMotion([] {
    //         intake::set(intake::intake_disabled);
    //         matchloader::up();
    //     })
    //     .drive_chainErrorTolerance(8_in)
    //     .setChainTime(0_sec)
    //   // .drive_minVolt(0.5_volt)
    //   | chain;

    mb.turnTo(25, 60).reverse() | chain;

    // go to other side
    mb.moveTo(25, 60)
        .reverse()
        .drive_chainErrorTolerance(7_in)
        .setChainTime(30_msec)
        .drive_minVolt(0.2_volt) |
      chain;
    // get on same y
    mb.moveTo(42, long_goal + 0.2_in).reverse().only_y(true) | chain;

    score_long_goal(1, 1, 2.5_sec);

    pros::Task([] {
        // allow last balls to score
        pros::delay(150);
        intake::out();
        pros::delay(200);
        intake::in();
    });

    matchload(1, 1, 2.5_sec);

    score_long_goal(1, 1, 3.0_sec, true);
    matchloader::up();

    // let it score last one
    intake::score_long();

    // go towards park
    mb.boomerang(62, 19, 270).lead(0.3).drive_maxVolt(0.5_volt) | async;
    pros::delay(300);
    intake::out();
    pros::delay(400);
    intake::score_long();
    async.wait();

    // disable horizontal odom
    horizontal_tracker.setDisabled(true);
    vertical_tracker.setDisabled(true);
    odom_retract::retractOdom();

    // here the thresholds likely need to be bigger for reseting to happen
    // we later reset them back to what they should be
    // setMaxDistanceThresholdAll(10_in);
    // setSmootherAlphas(std::nullopt, smoother_config.alpha_y * 2.0);

    drivetrain.moveTank(0.2_volt, 0.25_volt);
    // move closer to the park slowly, also gives time for accurate start roll

    pros::delay(400);

    // now going for park
    intake::in();

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

    // reset the max distance as we hope we have the right location
    // resetMaxDistanceThresholdAll();
    // resetSmootherConfig();

    mb.moveTo(31.5, -18.5).drive_maxVolt(1.0_volt) | run;
    mb.moveTo(23, -23.4).drive_maxVolt(0.6_volt) | run;
    matchloader::up();

    // go to bottom goal

    mb.turnTo(centerTopGoalSecond.x, centerTopGoalSecond.y) | run;
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

    async.waitUntil(closeEnough({ 8_in, -8_in }, 6_in));

    // intake::set(intake::scoring_middle_bottom_balls);
    intake::set(intake::outtake_bottom_balls_open_middle);
    pros::delay(100);
    intake::set(intake::scoring_middle_bottom_balls);
    pros::delay(300);
    intake::set(intake::scoring_middle_bottom_balls_slow);

    start_time = now();
    bool bad_color = false;

    while (true) {
        bool timeout_done = timeoutDone(2900_msec, start_time);
        bad_color = intake::getMiddleDetectedColor() == alliance_t::blue;
        if (timeout_done || bad_color) break;

        pros::delay(10);
    }

    intake::set(intake::scoring_middle_top_balls_skills_fast);
    pros::delay(800);
    intake::set(intake::scoring_middle_top_balls_skills);
    pros::delay(1200);

    // exit any motions if the are somehow still executing
    async.exitAll();

    mb.moveTo(48, match3 - 1.7_in)
        .reverse()
        .drive_backwardsAccelSlew(0.05_volt) |
      async;
    pros::delay(400);
    intake::out();

    async.wait();

    matchloader::down();
    intake::in();

    pros::delay(100);

    mb.turnTo(make_matchloader_point(1, -1)) | run;
    matchload(1, -1, 2.5_sec);

    mb.moveTo(27, -59.5)
        .reverse()
        .executeAfterMotion([] {
            intake::set(intake::intake_disabled);
            matchloader::up();
        })
        .drive_chainErrorTolerance(8_in)
        .setChainTime(0_sec)
        .drive_maxVolt(0.5_volt) |
      chain;

    mb.turnTo(-25, -59).reverse() | chain;

    // go to other side
    mb.moveTo(-25, -59)
        .reverse()
        .drive_chainErrorTolerance(7_in)
        .setChainTime(30_msec)
        .drive_minVolt(0.2_volt) |
      chain;

    // get on same y
    mb.moveTo(-46, -long_goal - 1.3_in).reverse() | chain;

    score_long_goal(-1, -1, 2.5_sec);
    // intake::in();

    // manually add 3 degrees to orientation
    RobotSetPose({ RobotGetPose().x,
                   RobotGetPose().y,
                   RobotGetPose().orientation + 3_stDeg });

    pros::delay(20);

    pros::Task([] {
        // allow last balls to score
        pros::delay(150);
        intake::out();
        pros::delay(200);
        intake::in();
    });

    matchload(-1, -1, 2.5_sec);

    score_long_goal(-1, -1, 2.5_sec, true);
    matchloader::up();

    mb.boomerang(-62, -18, 90)
        .lead(0.3)
        .drive_maxVolt(0.5_volt)
        .drive_minVolt(0.3_volt)
        .drive_largeErrorTolerance(5_in)
        .drive_toleranceDuration(0_sec) |
      run;

    horizontal_tracker.setDisabled(true);
    odom_retract::retractOdom();
    // pros::delay(300);

    intake::in();

    drivetrain.moveTank(0.2_volt, 0.25_volt);
    // move closer to the park slowly, also gives time for accurate start roll

    pros::delay(400);

    //
    // drivetrain.moveTank(0.6_volt, 0.65_volt);
    // pros::delay(700);
    // drivetrain.moveTank(0.4_volt, 0.5_volt);
    // pros::delay(210);
    //
    // // stop the robot
    // drivetrain.moveTank(0.0_volt, 0.0_volt);

    drivetrain.moveTank(0.53_volt, 0.63_volt);
    pros::delay(300);
    drivetrain.moveTank(0.4_volt, 0.5_volt);
    pros::delay(430);
    // drivetrain.moveTank(0.2_volt, 0.3_volt);
    // pros::delay(800);
    drivetrain.moveTank(0_volt, 0_volt);
}

} // namespace ninesix_skills
