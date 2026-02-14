/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "apis.h"
//
#include "auton_globals.h"
#include "autos.h"
#include "blazing/executor.hpp"
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
#include <optional>

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
            // .turn_vel_kd(linear_angular_vel_pid.get_kd() * 1.01)
            .turn_vel_kd(14.10)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_vel_mp_setMaxAccel(70_inps2) |
          async;

        Length matchload_start_distance = 13_in;

        auto custom_exit_condition = [&] -> bool {
            auto curr_pose = RobotGetPose();
            auto error = (target_Point - curr_pose);
            auto local_error = error.rotatedBy(-curr_pose.orientation);

            bool close = error.magnitude() <
                         // trigger only if closes to the matchloader
                         matchload_start_distance + 5_in;

            bool forwards_close =
              units::abs(local_error.x) < matchload_start_distance;

            // use forwards error and
            return close && forwards_close;
        };

        auto wait_result = async.waitOr(custom_exit_condition, 3_sec);

        if (wait_result == blazing::AsyncExecutorBase::motionFinished ||
            wait_result == blazing::AsyncExecutorBase::timeoutFinished) {
            // custom condition did not trigger, meaning we got stuck or
            // something else went wrong. Don't wait just exit
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
            auto curr_pose = RobotGetPose();
            bool x_close = units::abs(curr_pose.x) >= 27_in &&
                           units::abs(curr_pose.x) <= 29.5_in;
            bool y_close = units::abs(curr_pose.y) >= 43_in &&
                           units::abs(curr_pose.y) <= 51_in;
            //
            bool theta_close =
              units::abs(angleError(target_forwards_heading,
                                    curr_pose.orientation)) <= 25_stDeg;

            return x_close && y_close && theta_close;
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
            // aims for point
            // mb.moveTo(17_in * sign_x, 55.1_in * sign_y)
            mb.boomerang(17_in * sign_x,
                         55_in * sign_y,
                         // reverse of actual when scoring
                         target_forwards_heading)
                .reverse()
                // since point is farther away no point in trying this
                .drive_vel_minVel(50_inps)
                .lead(0.12)
                // .drive_chainErrorTolerance(0_in)
                // .drive_chainErrorTolerance(10_in)
                // .chainHalfcircleTolerance(std::nullopt)
                // use half circle exit for better exit conditions?
                // .chainHalfcircleTolerance(1_in, 5_in)
                .setChainTime(10_msec) |
              chain;

            // mb.turnTo(21.8_in, 47_in)
            mb.turnTo(target_backwards_heading)
                .reverse()
                .direction(AngularDirection::RIGHT)
                .radius(-10.5_in / 2)
                .timeout(2.6_sec) |
              chain;
        }

        chain.waitOr(exit_condition);

        // regardless of getting stuck or not we perform the same action

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

    RobotSetPose(-46.8, 15.514, 0);

    intake::in();

    mb.moveTo(-23.11, 17.613) | chain;

    mb.turnTo(-11.6, 11.0).reverse() | chain;
    mb.moveTo(-11.6, 11.0).reverse() | chain;

    // align tech
    mb.turnTo(135)
        .radius(-5_in)
        // infinite time motion
        .turn_toleranceDuration(100_sec)
        .turn_largeToleranceDuration(100_sec)
        .timeout(4_sec) |
      chain;

    chain.waitUntil(closeEnough({ -11.6_in, 11.0_in }, 4_in));

    // start scoring
    intake::score_middle();

    pros::delay(2000);
    chain.exitAll();

    pros::Task([] {
        // need a bit of time for the last ball on the long goal
        // before starting to intake
        pros::delay(200);
        intake::in();
    });

    mb.moveTo(-43.02, 46).only_y(true)
      // .drive_errorTolerance(0.5_in)
      // .drive_toleranceDuration(0_sec)
      | run;

    matchload(-1, 1, 1.5_sec);

    // go away from matchloader
    mb.moveTo(-30, 56.5)
        .reverse()
        .closeThreshold(4_in)
        .drive_vel_minVel(50_inps)
        // can sacrifice cross track here for speed (???)
        // .customAngularLinearFunc([](Angle angle) -> double {
        //     return units::cos(angle);
        // })
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

    // --- BLUE PARK --- //
    // sprint straight towards second park
    mb.moveTo(58, 20) | chain;
    mb.turnTo(270).radius(4_in).executeBeforeMotion([] {
        // retract to go over park
        // doesn't matter for turn since its heading based
        // doing it during the motion makes it so we don't ahve to wait for the
        // odom to lift up
        odom_retract::retractOdom();
        horizontal_tracker.setDisabled(true);
    }) |
      chain;
    chain.wait();

    drivetrain.moveTank(0.2_volt, 0.25_volt);
    // move closer

    pros::delay(400);

    // now going for park
    intake::in();

    // get over first part of park
    drivetrain.moveTank(0.53_volt, 0.63_volt);
    pros::delay(300);
    drivetrain.moveTank(0.4_volt, 0.5_volt);
    pros::delay(1200);
    drivetrain.moveTank(0.2_volt, 0.3_volt);
    pros::delay(600);
    // pull down matchloader only at the pure end
    matchloader::down();

    // enable odom again
    odom_retract::lowerOdom();
    horizontal_tracker.setDisabled(false);

    // move backwards towards the park again
    // also gives time to relocalize
    // drivetrain.moveTank(-0.3_volt, -0.3_volt);
    // pros::delay(200);

    mb.turnTo(180) | run;

    // reset our pose
    LaserResets({ &left_laser_model, &back_laser_model });

    // use inertial from before, turn left
    // before going pull it up
    matchloader::up();
    mb.moveTo(40.625, -17.751) | chain;
    mb.turnTo(23.11, -17.751) | chain;
    mb.moveTo(23.11, -17.713) | chain;

    mb.turnTo(11.6, -11.0).reverse() | chain;
    mb.moveTo(11.6, -11.0).reverse() | chain;

    // align tech
    mb.turnTo(135)
        .reverse()
        .radius(-5_in)
        // infinite time motion
        .turn_toleranceDuration(100_sec)
        .turn_largeToleranceDuration(100_sec)
        .timeout(4_sec) |
      chain;

    chain.waitUntil(closeEnough({ -11.6_in, 11.0_in }, 4_in));
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
    mb.moveTo(30.692, -57)
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
        horizontal_tracker.setDisabled(true);
    }) |
      chain;
    chain.wait();

    drivetrain.moveTank(0.5_volt, 0.5_volt);
    pros::delay(900);
    drivetrain.moveTank(0.0_volt, 0.0_volt);

    // cinema
}

} // namespace states_skills
