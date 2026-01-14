/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "apis.h"
//
#include "autos.h"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/device_globals.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/wings.h"
#include "units/Angle.hpp"
#include <iostream>

// do not do anything outside here!

namespace seven_ball {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

    auto make_machloader_point = [&](units::V2FPosition target,
                                     Length distance) -> units::V2FPosition {
        auto target_angle = target.angleTo(RobotGetPose());

        return target + distance * (RobotGetPose() - target).normalize();
        // return target + units::V2Position::fromPolar(target_angle, distance);
    };

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    Length match1 = normal_match;

    units::V2Position centerTopGoalFirst = { -7.9_in, 7.4_in };
    units::V2Position centerBottomGoalFirst = { -11_in, -11_in };

    bool winging = true;
    bool fast_wing = false;
    Voltage slow_wing_speed = 0.5_volt;

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    auto matchload = [&](double sign_x,
                         double sign_y,
                         Time moveTimeout,
                         Time matchloading_time,
                         Length target_distance = 7.5_in,
                         auto kp = linear_pid.get_kp() * 0.9,
                         auto kd = linear_pid.get_kd() * 1.2,
                         Voltage max_volt = 1.0_volt) {
        // turn to and go to matchloader
        mb.turnTo(67.4_in * sign_x, normal_match * sign_y)
            .executeBeforeMotion([] {
                // pros::Task([] {
                //     pros::delay(100);
                matchloader::down();
                // });
            })
          // .turn_toleranceDuration(20_msec)
          | run;
        // chain.wait();

        // auto thingy = chain.getCurrentIndex();

        auto match_point =
          make_machloader_point({ 67.4_in * sign_x, normal_match * sign_y },
                                target_distance);

        mb.moveTo(match_point.x, match_point.y)
            .timeout(moveTimeout)
            .drive_maxVolt(max_volt)
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

    auto score_long_goal = [&](double sign_x,
                               double sign_y,
                               Time score_time,
                               Voltage max_volt = 1.0_volt) {
        // turn to goal, reversed
        mb.turnTo(25_in * sign_x, long_goal * sign_y).reverse() | chain;
        mb.moveTo(25_in * sign_x, long_goal * sign_y)
            .reverse()
            .k_lat(0.0)
            .drive_maxVolt(max_volt) |
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
            .turn_kp(turn_drive_pid.get_kp() * 2)
            .turn_kd(turn_drive_pid.get_kd() * 0.5) |
          async;
        pros::delay(to_msec(score_time - 300_msec));
        async.exitAll();
    };

    double angle = bl ? 0 : 0;

    intake::in();

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::up();

    RobotSetPose(-47.2, 14.9 * l, angle);
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // if (!bl) {
    //     LaserResets({
    //       &right_laser_model,
    //       &back_laser_model,
    //     });
    // }
    //
    // std::cout << std::format("reset {:.2f},{:.2f},{:.2f}",
    //                          RobotGetPose().x.convert(in),
    //                          RobotGetPose().y.convert(in),
    //                          RobotGetPose().orientation.convert(deg))
    //           << std::endl;

    // only intake bottom balls to save time
    intake::setColorSortEnabled(false);
    intake::in();

    mb.moveTo(-23.5, 21.4 * l).drive_maxVolt(0.45_volt) | chain;

    chain.waitUntil(closeEnough({ -25.5_in, 23.4_in * l }, 9.0_in));
    matchloader::down();

    chain.wait();

    mb.moveTo(-48, (match1)*l).drive_maxVolt(0.6_volt).reverse() | run;

    // its a run here, so we can do these things
    intake::in();
    // matchloader::down();
    // LaserResets({ &front_laser_model });

    matchload(-1,
              l,
              1_sec,
              600_msec,
              7.0_in,
              linear_pid.get_kp() * 0.7,
              linear_pid.get_kd() * 0.5,
              1.0_volt);

    score_long_goal(-1, l, 1.5_sec, 0.6_volt);
    matchloader::up();

    // mb.turnTo(-80, (match1)*l) | chain;
    // mb.moveTo(-61.1, (match1)*l)
    //     .drive_maxVolt(0.65_volt)
    //     .timeout(1.3_sec)
    //     .executeBeforeMotion([] {
    //         pros::delay(400);
    //         intake::in();
    //     }) |
    //   chain;
    // chain.wait();
    // // pros::delay(100);
    //
    // mb.turnTo(-24_in, long_goal * l).reverse() | run;
    // // mb.turnTo(0).reverse() | run;
    // mb.moveTo(-26.5_in, (match1 * l))
    //     .reverse()
    //     .timeout(1.4_sec)
    //     .k_lat(0.0)
    //
    //     // changed today!
    //     // .turn_kp(angular_pid.get_kp() * 0.5)
    //     // .turn_kd(angular_pid.get_kd() * 0.5)
    //     .drive_backwardsAccelSlew(0.4_volt)
    //     //
    //
    //     .drive_maxVolt(0.65_volt)
    //     .closeThreshold(8_in)
    //     .executeAfterMotion([] {
    //         drivetrain.moveTank(-0.3_volt, -0.3_volt);
    //     }) |
    //   chain;

    // chain.waitUntil(closeEnough({ -32_in, long_goal * l }, 4_in));
    // intake::score_long();

    if (winging) {
        if (bl) {
            mb.moveTo(-35.737, 37.1) | chain;
            mb.turnTo(0).reverse() | chain;
            wings::down();
            mb.boomerang(-9, 37, 0)
                .reverse()
                .drive_maxVolt(fast_wing ? 1.0_volt : slow_wing_speed)
                .drive_toleranceDuration(100_sec)
                .drive_largeToleranceDuration(100_sec)
                .timeout(100_sec) |
              chain;

        } else {
            mb.moveTo(-35.737, -36.7) | chain;
            mb.turnTo(0) | chain;
            wings::down();
            // mb.moveTo(-9, -37.2)
            //     .drive_maxVolt(fast_wing ? 1.0_volt : slow_wing_speed)
            //     .drive_toleranceDuration(100_sec)
            //     .drive_largeToleranceDuration(100_sec)
            //     .timeout(100_sec) |
            //   run;

            mb.boomerang(-8.7, -37.2, 0)
                .drive_maxVolt(fast_wing ? 1.0_volt : slow_wing_speed)
                .drive_toleranceDuration(100_sec)
                .drive_largeToleranceDuration(100_sec)
                .timeout(100_sec) |
              chain;
        }
    }

    // pros::delay(500);
    // // start_time = now();
    //
    // intake::set(intake::scoring_long_top_balls_outake_bottom);
    //
    // pros::delay(1700);
}

} // namespace seven_ball
