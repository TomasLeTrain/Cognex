/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "apis.h"
//
#include "auton_globals.h"
#include "autos.h"
#include "blazing/utils.hpp"
#include "globals.h"
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
#include <iostream>

// do not do anything outside here!

namespace states_skills {

void pre_auton() {
    // set the robot state to match expectations
    // done in case driver or such is run before auto
    wings::up();
    odom_retract::lowerOdom();
    matchloader::up();

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    intake::setAutonColorSort(false);
}

void run_auton() {
    // runs before anything else
    pre_auton();

    // do whatever you want here
    units::V2Position centerTopGoalFirst = { -9.5_in, 8.0_in };

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    auto make_matchloader_point = [](double sign_x,
                                     double sign_y) -> units::V2Position {
        Length normal_match = 46.7_in;
        return { 67.4_in * sign_x, normal_match * sign_y };
    };

    auto matchload = [make_matchloader_point](double sign_x,
                                              double sign_y,
                                              Time matchload_time) {
        auto make_machloader_pose = [](units::V2FPosition target,
                                       Length distance) -> units::Pose {
            // auto target_angle = target.angleTo(RobotGetPose());
            auto final_point =
              target + distance * (RobotGetPose() - target).normalize();

            return units::Pose { final_point, final_point.angleTo(target) };
        };

        Length normal_match = 46.7_in;
        units::V2Position target_Point = make_matchloader_point(sign_x, sign_y);
        Length target_dist = 11_in;

        auto func = [&] -> units::Pose {
            return make_machloader_pose(target_Point, target_dist);
        };

        // make sure we are matchloading
        matchloader::down();

        Time motion_start_time = now();

        mb.moveTo(func)
            // if it takes longer it most likely got stuck
            .timeout(1.5_sec)
            .drive_vel_accelSlew(110_inps2)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_vel_mp_setMaxAccel(70_inps2) |
          async;

        Length matchload_start_distance = 13_in;

        auto custom_exit_condition = [&] -> bool {
            // use forwards error and
            return (target_Point - RobotGetPose()).magnitude() <
                     // trigger only if closes to the matchloader
                     matchload_start_distance + 5_in &&
                   units::abs((target_Point - RobotGetPose())
                                .rotatedBy(-RobotGetPose().orientation)
                                .x) < matchload_start_distance;
        };

        bool motion_finished = false;

        while (true) {
            bool exit_now = custom_exit_condition();
            motion_finished = async.numQueuedMotions() == 0;

            if (motion_finished || exit_now) break;
            pros::delay(10);
        }

        if (motion_finished) {
            // motion finished before matchload start time, we likely got stuck
            // and should stop any more matchloading time
            async.exitAll();
        } else {
            // got to matcloader successfully, start matchloading
            async.exitAll();
            // passive voltage forwards since motion might oscilate
            drivetrain.moveTank(0.13_volt, 0.13_volt);
            pros::delay(to_msec(matchload_time));
        }
    };

    auto score_long_goal = [](double sign_x,
                              double sign_y,
                              Time score_time,
                              bool with_swing = false) {
        // turn to goal, reversed
        Length long_goal = 47.05_in;

        auto target_backwards_heading = sign_x == -1 ? 0_stDeg : 180_stDeg;
        auto target_forwards_heading = sign_x == -1 ? 180_stDeg : 0_stDeg;

        units::Pose target_pose = { 24_in * sign_x,
                                    long_goal * sign_y,
                                    target_backwards_heading };

        auto exit_condition = [&] -> bool {
            // use forwards error and

            auto curr_pose = RobotGetPose();
            bool x_close = units::abs(curr_pose.x) >= 27_in &&
                           units::abs(curr_pose.x) <= 29.5_in;
            bool y_close = units::abs(curr_pose.y) >= 43_in &&
                           units::abs(curr_pose.y) <= 51_in;
            //
            bool theta_close =
              units::abs(angleError(target_forwards_heading,
                                    curr_pose.orientation)) < 25_stDeg;

            return x_close && y_close && theta_close;
            // return x_close;
        };

        // turn towards 24, settle at 48
        if (!with_swing) {
            mb.moveTo(target_pose)
                .drive_vel_mp_setMaxAccel(110_inps2)
                .only_x(true, 28_in)
                .closeThreshold(10_in)
                .timeout(2_sec)
                .reverse() |
              chain;
        } else {
            mb.moveTo(17_in * sign_x, 54.5_in * sign_y)
                .reverse()
                .drive_vel_minVel(50_inps)
                .setChainTime(0_sec) |
              chain;

            // mb.turnTo(21.8_in, 47_in)
            mb.turnTo(target_backwards_heading)
                .reverse()
                .direction(AngularDirection::RIGHT)
                .radius(-10.5_in / 2)
                .timeout(2.6_sec) |
              chain;
        }

        chain.waitUntil(exit_condition);

        intake::score_long();
        // let move to point settle a bit
        pros::delay(100);
        chain.exitAll();
        // queue aligning motion
        mb.turnTo(target_forwards_heading).radius(-4.0_in) | chain;

        pros::delay(units::max(to_msec(score_time) - 100, 0));
        chain.exitAll();
    };

    /* START AUTON */

    RobotSetPose(-46.7, 0.0, 180);

    // start by going into the park

    // disable odom
    odom_retract::retractOdom();
    horizontal_tracker.setDisabled(true);

    drivetrain.moveTank(1.0_volt, 1.0_volt);
    pros::delay(200);
    drivetrain.moveTank(0.5_volt, 1.5_volt);
    pros::delay(200);
    drivetrain.moveTank(0.2_volt, 0.2_volt);

    Time first_park_start_time = now();

    while (true) {
        Length measured = from_mm(front_distance.get());

        bool timeout_done = timeoutDone(3_sec, first_park_start_time);

        bool distance_exit = false;

        if (measured <= 40_mm) {
            // reading intake, ignore
        } else {
            if (measured <= 130_mm) {
                distance_exit = true;
            }
        }

        if (timeout_done || distance_exit) break;
        pros::delay(10);
    }

    drivetrain.moveTank(-0.5_volt, -0.5_volt);
    pros::delay(800);

    // enable odom again
    odom_retract::lowerOdom();
    horizontal_tracker.setDisabled(false);

    // go back again towards the park to reset?
    setSmootherAlphas(0.3, 0.2);
    front_laser_model.setMaxDistanceDifference(7_in);
    drivetrain.moveTank(0.2_volt, 0.2_volt);
    pros::delay(500);
    resetSmootherConfig();
    resetMaxDistanceThresholdAll();

    intake::in();

    // swing back a bit
    drivetrain.moveTank(0_volt, -1_volt);
    pros::delay(300);
    mb.moveTo(-39, -19.2).reverse() | chain;

    mb.moveTo(-22.414, -17.062).drive_vel_minVel(20_inps) | chain;
    mb.moveTo(-11.097, -11.691).closeThreshold(4_in).executeAfterMotion([] {
        intake::score_bottom();
    }) |
      chain;
    // align tech
    mb.turnTo(45).radius(5_in) | chain;

    chain.wait();

    pros::delay(3000);

    drivetrain.moveTank(-1_volt, -1_volt);
    pros::delay(170);
    intake::in();

    // got towards matchloader at other end
    // mb.turnTo(-43.02, 44.349) | run;
    mb.moveTo(-43.02, 45.5)
        .only_y(true)
        // .drive_errorTolerance(0.5_in)
        .drive_toleranceDuration(0_sec) |
      run;

    matchload(-1, 1, 1.5_sec);

    // go away from matchloader
    mb.moveTo(-28, 56)
        .reverse()
        .closeThreshold(4_in)
        .drive_vel_minVel(50_inps)
        // can sacrifice cross track here for speed
        .customAngularLinearFunc([](Angle angle) -> double {
            return units::cos(angle);
        })
        .executeAfterMotion([] {
            // up matchloader here to avoid getting stuck in the swing
            matchloader::up();
        })
        .setChainTime(0_sec) |
      chain;

    // swing is chained, so no waiting here
    // uses swing to score on long
    score_long_goal(1, 1, 2_sec, true);

    pros::Task([] {
        // need a bit of time for the last ball on the long goal
        // before starting to intake
        pros::delay(200);
        intake::in();
    });

    matchload(1, 1, 1.5_sec);

    score_long_goal(1, 1, 2_sec);

    matchloader::up();

    pros::Task([] {
        // need a bit of time for the last ball on the long goal
        // before starting to intake
        pros::delay(200);
        intake::in();
    });

    // --- SECOND PARK --- //
    // sprint straight towards second park
    mb.moveTo(41.142, 0) | run;
    mb.turnTo(0) | run;

    // disable odom
    odom_retract::retractOdom();
    horizontal_tracker.setDisabled(true);

    // align against the park while having some speed
    drivetrain.moveTank(0.3_volt, 0.3_volt);
    pros::delay(300);

    drivetrain.moveTank(0.5_volt, 0.5_volt);
    pros::delay(700);
    drivetrain.moveTank(0.2_volt, 0.2_volt);

    Time second_park_start_time = now();

    while (true) {
        Length measured = from_mm(front_distance.get());

        bool timeout_done = timeoutDone(3_sec, second_park_start_time);

        bool distance_exit = false;

        if (measured <= 40_mm) {
            // reading intake, ignore
        } else {
            if (measured <= 130_mm) {
                distance_exit = true;
            }
        }

        if (timeout_done || distance_exit) break;
        pros::delay(10);
    }
    drivetrain.moveTank(-0.5_volt, -0.5_volt);
    pros::delay(800);

    // enable odom again
    odom_retract::lowerOdom();
    horizontal_tracker.setDisabled(false);

    // go back again towards the park to reset?
    setSmootherAlphas(0.3, 0.2);
    front_laser_model.setMaxDistanceDifference(7_in);
    drivetrain.moveTank(0.2_volt, 0.2_volt);
    pros::delay(500);
    resetSmootherConfig();
    resetMaxDistanceThresholdAll();

    // swing back a bit
    drivetrain.moveTank(-1.0_volt, -1.0_volt);
    pros::delay(200);

    // use inertial from before, turn left
    mb.turnTo(29.545, -16.952) | chain;
    mb.moveTo(29.545, -16.952) | chain;
    mb.turnTo(12.574, -12.401).reverse() | chain;
    mb.moveTo(11.574, -11.5).reverse() | chain;

    auto top_middle_scoring_indx = chain.getCurrentIndex();

    // align tech
    mb.turnTo(135)
        .reverse()
        .radius(-5_in)
        // infinite time motion
        .turn_toleranceDuration(100_sec)
        .turn_largeToleranceDuration(100_sec)
        .timeout(4_sec) |
      chain;

    chain.waitUntilIndex(top_middle_scoring_indx);
    // start scoring?
    intake::score_middle();
    pros::delay(3000);
    chain.exitAll();

    mb.moveTo(43.02, -46)
        .only_y(true)
        .drive_errorTolerance(1.3_in)
        .drive_toleranceDuration(0_sec)
        .executeBeforeMotion([] {
            // wait a bit before starting to intake again to not interrept balls
            // that were just scored
            pros::delay(300);
            intake::in();
        }) |
      run;

    matchload(1, -1, 1.5_sec);

    // go away from matchloader
    mb.moveTo(30.692, -56)
        .reverse()
        .closeThreshold(4_in)
        .drive_vel_minVel(50_inps)
        // can sacrifice cross track here for speed
        .customAngularLinearFunc([](Angle angle) -> double {
            return units::cos(angle);
        })
        .executeAfterMotion([] {
            // up matchloader here to avoid getting stuck in the swing
            matchloader::up();
        })
        .setChainTime(0_sec) |
      chain;

    // uses swing to score on long
    score_long_goal(-1, -1, 2_sec, true);

    pros::Task([] {
        // need a bit of time for the last ball on the long goal
        // before starting to intake
        pros::delay(200);
        intake::in();
    });

    matchload(-1, -1, 1.5_sec);

    score_long_goal(-1, -1, 2_sec);

    // intake any balls in the way and shoot them out on the way to the park
    intake::score_long();

    // finally park

    matchloader::up();
    mb.moveTo(-59, -20) | chain;
    mb.turnTo(90).radius(4_in).executeBeforeMotion([] {
        // retract to go over park
        // doesn't matter for turn since its heading based
        // doing it during the motion makes it so we don't ahve to wait for the
        // odom to lift up
        odom_retract::retractOdom();
    }) |
      chain;
    chain.wait();

    drivetrain.moveTank(0.5_volt, 0.5_volt);
    pros::delay(900);
    drivetrain.moveTank(0.0_volt, 0.0_volt);

    // cinema
}

} // namespace states_skills
