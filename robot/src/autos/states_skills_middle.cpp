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
#include "units/units.hpp"
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

    auto moveVel = [](Voltage left_vol, Voltage right_vol, Time timeout) {
        Time start_time = now();
        decltype(controllers) curr_controllers = controllers;
        double left_pct = left_vol.internal();
        double right_pct = right_vol.internal();

        double wanted_linear = (left_pct + right_pct) / 2.0;
        double wanted_angular = (right_pct - left_pct) / 2.0;

        LinearVelocity max_vel = 76_inps;
        AngularVelocity max_ang_vel = rad * max_vel / (10.5_in * 0.5);

        DifferentialSpeeds target { max_vel * wanted_linear,
                                    max_ang_vel * wanted_angular };

        while (true) {
            bool timeout_done = timeoutDone(timeout, start_time);

            auto result =
              curr_controllers.velocity_feedforward.update(target, 20_msec);
            drivetrain.moveTank(result.left_voltage, result.right_voltage);

            if (timeout_done) break;
            pros::delay(20);
        }
    };

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

        // 11 is barely achievable - 0.1 less than achievable
        Length target_dist = 10.9_in;

        auto func = [&] -> units::Pose {
            return make_machloader_pose(target_Point, target_dist);
        };

        // make sure we are matchloading
        matchloader::down();

        Time motion_start_time = now();

        std::cout << "func: " << func().x.convert(in) << " "
                  << func().y.convert(in) << " "
                  << func().orientation.convert(deg) << std::endl;

        mb.turnTo(func()) | run;
        // mb.moveTo(func)

        auto func_point = func();

        std::cout << func_point.x.convert(in) << " " << func_point.y.convert(in)
                  << " " << func_point.orientation.convert(deg) << std::endl;
        pros::delay(200);

        mb.moveTo(func_point)
            // if it takes longer it most likely got stuck
            .timeout(1.5_sec)
            .drive_vel_accelSlew(110_inps2)
            // .turn_vel_kd(linear_angular_vel_pid.get_kd() * 1.01)
            .drive_toleranceDuration(100_sec)
            .drive_largeToleranceDuration(100_sec)
            .drive_vel_mp_setMaxAccel(70_inps2)
          // .executeBeforeMotion([&] {
          //     std::cout << "after turn pos: " << RobotGetPose().x.convert(in)
          //               << " " << RobotGetPose().y.convert(in) << " "
          //               << RobotGetPose().orientation.convert(deg)
          //               << std::endl;
          //     std::cout << "after turn func: " << func().x.convert(in) << " "
          //               << func().y.convert(in) << " "
          //               << func().orientation.convert(deg) << std::endl;
          // })
          | async;
        async.wait();

        // Length matchload_start_distance = 13_in;
        //
        // auto custom_exit_condition = [&] -> bool {
        //     auto curr_pose = RobotGetPose();
        //     auto error = (target_Point - curr_pose);
        //     auto local_error = error.rotatedBy(-curr_pose.orientation);
        //
        //     bool close = error.magnitude() <
        //                  // trigger only if closes to the matchloader
        //                  matchload_start_distance + 5_in;
        //
        //     bool forwards_close =
        //       units::abs(local_error.x) < matchload_start_distance;
        //
        //     // use forwards error and
        //     return close && forwards_close;
        // };
        //
        // auto wait_result = async.waitOr(custom_exit_condition, 3_sec);
        //
        // if (wait_result == blazing::AsyncExecutorBase::motionFinished ||
        //     wait_result == blazing::AsyncExecutorBase::timeoutFinished) {
        //     // custom condition did not trigger, meaning we got stuck or
        //     // something else went wrong. Don't wait just exit
        //     async.exitAll();
        // } else {
        //     // got to matcloader successfully, start matchloading
        //     async.exitAll();
        //     // passive voltage forwards since motion might oscilate
        //     drivetrain.moveTank(0.13_volt, 0.13_volt);
        //     pros::delay(to_msec(matchload_time));
        // }
    };

    auto score_long_goal = [](double sign_x,
                              double sign_y,
                              Time score_time,
                              bool with_swing = false,
                              Time slow_score_time = 0_sec) {
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
            mb.boomerang(16.4_in * sign_x,
                         55.2_in * sign_y,
                         // reverse of actual when scoring
                         target_forwards_heading)
                .reverse()
                // since point is farther away no point in trying this
                // .drive_vel_minVel(50_inps)
                .closeThreshold(4_in)
                .lead(0.12)
              // .drive_chainErrorTolerance(0_in)
              // .drive_chainErrorTolerance(10_in)
              // .chainHalfcircleTolerance(std::nullopt)
              // use half circle exit for better exit conditions?
              // .chainHalfcircleTolerance(1_in, 5_in)
              | chain;

            chain.wait();

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

        Time no_color_time = 100_msec;

        intake::score_long();
        // let move to point settle a bit
        pros::delay(to_msec(no_color_time));
        chain.exitAll();
        // queue aligning motion
        mb.turnTo(target_forwards_heading).radius(-4.0_in) | chain;

        Time new_score_time = units::max(score_time - no_color_time, 1_msec);

        if (to_msec(slow_score_time) < 2.0) {
            // not active
            pros::delay(to_msec(new_score_time));
        } else {
            // if not zero
            Time start_time = now();
            bool timeout_done = false;
            while (true) {
                timeout_done = timeoutDone(new_score_time, start_time);
                bool color_found = intake::colors::getMiddleColor()
                                     .transform([](auto color) {
                                         return color == alliance_t::blue;
                                     })
                                     .value_or(false);
                if (color_found || timeout_done) break;
                pros::delay(10);
            }

            if (timeout_done) {
                // never found blue balls, just give up and move on
            } else {
                // score slighlty slower
                intake::score_long(1.0, 0.3);
                pros::delay(to_msec(slow_score_time));
            }
        }

        chain.exitAll();
    };

    /* START AUTON */

    RobotSetPose(-46.8, 15, 0);

    intake::in();

    mb.moveTo(-23, 18.1) | run;

    mb.turnTo(0, 0).reverse() | chain;
    mb.moveTo(-11.6, 11.0).reverse()
      // .executeAfterMotion([] {
      //       drivetrain.moveTank(-0.15_volt, -0.15_volt);
      //   })
      | chain;

    // ???
    mb.distanceAtHeading(2_in, 135) | chain;

    // align tech
    // mb.turnTo(135)
    //     .radius(-5_in)
    //     // infinite time motion
    //     .turn_toleranceDuration(100_sec)
    //     .turn_largeToleranceDuration(100_sec)
    //     .timeout(4_sec) |
    //   chain;

    chain.waitUntil(closeEnough({ -11.6_in, 11.0_in }, 3.5_in));

    // start scoring
    // give the other ball time to get to the top
    pros::delay(500);
    // score
    intake::score_middle();

    pros::delay(1300);
    chain.exitAll();

    pros::Task([] {
        // need a bit of time for the last ball on the long goal
        // before starting to intake
        pros::delay(210);
        intake::out();
        // let outtake a bit to clear possible jam
        pros::delay(300);
        // intake
        intake::in();
    });

    mb.moveTo(-43.02, 46).only_y(true)
      // .drive_errorTolerance(0.5_in)
      // .drive_toleranceDuration(0_sec)
      | run;

    matchload(-1, 1, 2.0_sec);

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
        }) |
      chain;

    // swing is chained, so no waiting here
    // uses swing to score on long
    score_long_goal(1, 1, 2_sec, true);

    pros::Task([] {
        // need a bit of time for the last ball on the long goal
        // before starting to intake
        pros::delay(210);
        intake::out();
        // let outtake a bit to clear possible jam
        pros::delay(300);
        // intake
        intake::in();
    });

    matchload(1, 1, 2.0_sec);

    score_long_goal(1, 1, 2.0_sec, false, 1.2_sec);

    matchloader::up();

    //   pros::Task([] {
    //       // need a bit of time for the last ball on the long goal
    //       // before starting to intake
    //       // pros::delay(200);
    //       // intake::in();
    // // ej
    //   });

    // --- BLUE PARK --- //
    // sprint straight towards second park
    mb.moveTo(58, 20) | chain;
    mb.turnTo(270).radius(4_in) | chain;
    chain.wait();

    drivetrain.moveTank(0.2_volt, 0.25_volt);
    // move closer
    odom_retract::retractOdom();
    horizontal_tracker.setDisabled(true);

    pros::delay(330);

    // now going for park
    intake::in();

    // get over first part of park
    moveVel(0.48_volt, 0.58_volt, 350_msec);
    moveVel(0.38_volt, 0.48_volt, 1200_msec);
    moveVel(0.2_volt, 0.3_volt, 650_msec);

    // pull down matchloader only at the pure end
    drivetrain.moveTank(0_volt, 0_volt);
    matchloader::down();

    // enable odom again
    odom_retract::lowerOdom();
    horizontal_tracker.setDisabled(false);
    // give it time to matchload downwards
    pros::delay(100);

    // move backwards towards the park again
    // also gives time to relocalize
    // drivetrain.moveTank(-0.3_volt, -0.3_volt);
    // pros::delay(200);

    mb.turnTo(180) | run;

    // reset our pose to somewhere we know we are close to in case laser reset
    // doesn't work
    RobotSetPose({ RobotGetPose().x, -30_in, RobotGetPose().orientation });
    pros::delay(100);
    LaserResets({ &left_laser_model, &back_laser_model });

    matchloader::up();
    mb.moveTo(40.625, -18.0) | chain;
    mb.turnTo(23.11, -18.0) | chain;
    mb.moveTo(23.11, -18.0) | chain;

    mb.turnTo(11.6, -11.0).reverse() | chain;
    mb.moveTo(11.6, -11.0).reverse().executeAfterMotion([] {
        drivetrain.moveTank(-0.15_volt, -0.15_volt);
    }) |
      chain;

    // align tech
    // mb.turnTo(135)
    //     .reverse()
    //     .radius(-5_in)
    //     // infinite time motion
    //     .turn_toleranceDuration(100_sec)
    //     .turn_largeToleranceDuration(100_sec)
    //     .timeout(4_sec) |
    //   chain;

    chain.waitUntil(closeEnough({ 11.6_in, -11.0_in }, 4_in));
    // start scoring decently fast
    intake::score_middle();
    pros::delay(2000);

    {
        // score slow for 2 seconds
        Time scoring_slow_time = 2_sec;

        Time start_time = now();
        bool timeout_done = false;
        while (true) {
            timeout_done = timeoutDone(scoring_slow_time, start_time);
            bool color_found = intake::colors::getMiddleColor()
                                 .transform([](auto color) {
                                     return color == alliance_t::blue;
                                 })
                                 .value_or(false);
            if (color_found || timeout_done) break;

            // score slower
            intake::score_middle(1.0, 0.2);
            pros::delay(10);
        }

        if (timeout_done) {
            // found blue balls, stop intake immediately to stop scoring a blue
            intake::motors_disabled();
        } else {
            // scored all balls, nothing else to do
        }
    }

    chain.exitAll();

    mb.moveTo(43.02, -46)
        .only_y(true)
        .drive_errorTolerance(1.3_in)
        .drive_toleranceDuration(0_sec)
        .executeBeforeMotion([] {
            // wait a bit before starting to intake again to not interrept balls
            // that were just scored
            pros::delay(300);
            intake::out();
            // let outtake a bit to clear possible jam
            pros::delay(300);
            // intake
            intake::in();
        }) |
      run;

    matchload(1, -1, 2.0_sec);

    // go away from matchloader
    mb.moveTo(30.692, -56.5)
        .reverse()
        .closeThreshold(4_in)
        .drive_vel_minVel(50_inps)
        // can sacrifice cross track here for speed
        // .customAngularLinearFunc([](Angle angle) -> double {
        //     return units::cos(angle);
        // })
        .executeAfterMotion([] {
            // up matchloader here to avoid getting stuck in the swing
            matchloader::up();
        }) |
      chain;

    // uses swing to score on long
    score_long_goal(-1, -1, 2_sec, true);

    pros::Task([] {
        // need a bit of time for the last ball on the long goal
        // before starting to intake
        pros::delay(210);
        intake::out();
        // let outtake a bit to clear possible jam
        pros::delay(300);
        // intake
        intake::in();
    });

    matchload(-1, -1, 2.0_sec);

    // score_long_goal(-1, -1, 1.5_sec, false, 1000_msec);
    score_long_goal(-1, -1, 2.0_sec, false, 1.2_sec);

    // intake any balls in the way and shoot them out on the way to the park
    intake::score_long();

    // finally park
    matchloader::up();
    mb.moveTo(-58, -20) | chain;
    mb.turnTo(90).radius(4.3_in) | chain;
    chain.wait();

    // get close to it
    odom_retract::retractOdom();
    horizontal_tracker.setDisabled(true);
    drivetrain.moveTank(0.2_volt, 0.2_volt);
    pros::delay(300);

    // move into it
    drivetrain.moveTank(0.43_volt, 0.5_volt);
    pros::delay(900);
    drivetrain.moveTank(0.0_volt, 0.0_volt);

    // cinema
}

} // namespace states_skills
