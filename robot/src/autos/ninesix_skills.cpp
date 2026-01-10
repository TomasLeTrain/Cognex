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
    // TODO: make distance autmatic
    auto make_machloader_point = [&](units::V2FPosition target,
                                     Length distance) -> units::V2FPosition {
        auto target_angle = target.angleTo(RobotGetPose());

        return target + distance * (RobotGetPose() - target).normalize();
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

    units::V2Position centerTopGoalFirst = { -8.0_in, 7.4_in };
    units::V2Position centerTopGoalSecond = { 1_in, 0_in };
    units::V2Position centerBottomGoalFirst = { 12_in, 12_in };
    units::V2Position centerBottomGoalSecond = { -12_in, -11_in };

    intake::setSkillsMiddleScoring(true);
    intake::setColorSortEnabled(false);

    auto matchload = [&](double sign_x,
                         double sign_y,
                         Time moveTimeout,
                         Time matchloading_time,
                         Length target_distance = 7.5_in,
                         auto kp = linear_pid.get_kp() * 0.9,
                         auto kd = linear_pid.get_kd() * 1.2) {
        // turn to and go to matchloader
        mb.turnTo(67.4_in * sign_x, normal_match * sign_y)
            .executeBeforeMotion([] {
                // pros::Task([] {
                //     pros::delay(100);
                matchloader::down();
                // });
            })
            .turn_toleranceDuration(20_msec) |
          chain;
        chain.wait();

        // auto thingy = chain.getCurrentIndex();

        auto match_point =
          make_machloader_point({ 67.4_in * sign_x, normal_match * sign_y },
                                target_distance);

        mb.moveTo(match_point.x, match_point.y)
            .timeout(moveTimeout)
            .drive_kp(kp)
            .drive_kd(kd) |
          chain;
        auto motion_ind = chain.getCurrentIndex();

        // chain.waitUntilIndex(thingy);
        chain.waitUntil(closeEnough(match_point, 4_in));
        if (motion_ind == chain.getFinishedIndex()) {
            // motion timed out before this executed, means we likely won't
            // matchload?
        } else {
            pros::delay(to_msec(matchloading_time));
        }
        chain.exitAll();
        // matchloader::up();
    };

    auto score_long_goal = [&](double sign_x, double sign_y, Time score_time) {
        // turn to goal, reversed
        mb.turnTo(25_in * sign_x, long_goal * sign_y).reverse() | chain;
        mb.moveTo(25_in * sign_x, long_goal * sign_y).reverse().k_lat(0.0) |
          chain;

        chain.waitUntil(
          closeEnough({ 25_in * sign_x, long_goal * sign_y }, 7.5_in));
        intake::score_long();
        pros::delay(300);
        chain.exitAll();

        mb.boomerang(23_in * sign_x, long_goal * sign_y, sign_x == -1 ? 0 : 180)
            .reverse()
            .closeThreshold(100_in)
            .timeout(100_sec)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .turn_kp(angular_pid.get_kp() * 2)
            .turn_kd(angular_pid.get_kd() * 0.5) |
          async;
        pros::delay(to_msec(score_time - 300_msec));
        async.exitAll();
    };

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(inactive);

    RobotSetPose(-45.7, 15, 0);
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // only intake bottom balls to save time
    // intake::set(intake::intake_bottom_balls);
    intake::setColorSortEnabled(false);
    intake::set(intake::intake_bottom_balls);

    mb.moveTo(-30.5, 19.4) | run;

    mb.arc(centerTopGoalFirst.x, centerTopGoalFirst.y, 0.7)
        .turn_maxVolt(0.3_volt)
        .direction(AngularDirection::RIGHT)
        .executeBeforeMotion([] {
            right_motors.set_brake_mode_all(pros::MotorBrake::brake);
        })
        .executeAfterMotion([] {
            right_motors.set_brake_mode_all(pros::MotorBrake::hold);
        }) |
      run;

    mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | run;

    mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
        .k_lat(0.3)
        .drive_maxVolt(0.45_volt)
        .executeBeforeMotion([] {
            // pros::Task([] {
            //     matchloader::down();
            //     pros::delay(240);
            //     matchloader::up();
            // });
        }) |
      chain;

    chain.waitUntil(closeEnough({ -8_in, 8_in }, 5.5_in));
    intake::out();
    pros::delay(200);
    intake::set(intake::scoring_middle_bottom_balls);
    chain.exitAll();
    drivetrain.moveTank(0.1_volt, 0.2_volt);

    pros::delay(1200);

    mb.moveTo(-48, match1 - 0.0_in)
        .reverse()
        .only_y(true)
        .drive_maxVolt(0.7_volt)
        // .drive_toleranceDuration(20_msec)
        .executeAfterMotion([] {
            intake::in();
            matchloader::down();
        }) |
      run;

    matchload(-1,
              1,
              1_sec,
              2.0_sec,
              7.5_in,
              linear_pid.get_kp() * 0.9,
              linear_pid.get_kd() * 1.2);

    mb.moveTo(-30, 60)
        .reverse()
        .executeAfterMotion([] {
            intake::set(intake::intake_disabled);
            matchloader::up();
        })
        .drive_chainErrorTolerance(7_in)
        .setChainTime(0_sec)
        .drive_minVolt(0.5_volt) |
      chain;

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

    intake::in();
    matchload(1,
              1,
              1_sec,
              2_sec,
              7.5_in,
              linear_pid.get_kp() * 0.7,
              linear_pid.get_kd() * 1.0);

    score_long_goal(1, 1, 2.5_sec);
    matchloader::up();

    // let it score last one
    intake::score_long();

    // go towards park
    mb.boomerang(62, 19, 270).lead(0.3).drive_maxVolt(0.5_volt) | run;

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

    mb.moveTo(31.5, -18.5).drive_maxVolt(0.5_volt) | run;
    mb.moveTo(23, -23.4).drive_maxVolt(0.3_volt) | run;
    matchloader::up();

    // reset the max distance as we hope we have the right location
    resetMaxDistanceThresholdAll();
    resetSmootherConfig();

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

    intake::set(intake::scoring_middle_bottom_balls);

    start_time = now();
    bool bad_color = false;

    while (true) {
        bool timeout_done = timeoutDone(2000_msec, start_time);
        bad_color = intake::getMiddleDetectedColor() == alliance_t::blue;
        if (timeout_done || bad_color) break;

        pros::delay(10);
    }
    intake::set(intake::scoring_middle_top_balls_skills_fast);
    pros::delay(1300);
    intake::set(intake::scoring_middle_top_balls_skills_slow);
    pros::delay(800);

    // exit any motions if the are somehow still executing
    async.exitAll();

    mb.moveTo(48, match3 - 1.4_in)
        .reverse()
        .drive_backwardsAccelSlew(0.05_volt) |
      async;
    pros::delay(400);
    intake::out();

    async.wait();

    matchloader::down();
    intake::in();

    matchload(1,
              -1,
              1_sec,
              2_sec,
              7.5_in,
              linear_pid.get_kp() * 0.9,
              linear_pid.get_kd() * 1.2);

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
        .drive_minVolt(0.2_volt) |
      chain;

    // get on same y
    mb.moveTo(-46, -long_goal - 1.3_in).reverse() | chain;

    score_long_goal(-1, -1, 2.5_sec);
    intake::in();

    matchload(-1,
              -1,
              1_sec,
              2_sec,
              7.5_in,
              linear_pid.get_kp() * 0.5,
              linear_pid.get_kd() * 0.8);

    score_long_goal(-1, -1, 2.5_sec);
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
