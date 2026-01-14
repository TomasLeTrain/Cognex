/**
 * @file
 * @brief auto file template. copy paste this file, change the name and then add
 * it to "autos.h"
 */

#include "apis.h"
//
#include "autos.h"
#include "blazing/utils.hpp"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/device_globals.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/wings.h"
#include "units/Angle.hpp"
#include <iostream>

// do not do anything outside here!

namespace elims_nineball_split {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

    auto closeEnough = [](units::V2Position target,
                          Length threshold) -> std::function<bool()> {
        return [target, threshold] -> bool {
            return RobotGetPose().distanceTo(target) < threshold;
        };
    };

    Length long_goal = 46.9_in;
    Length normal_match = 46.7_in;

    Length match1 = normal_match;

    units::V2Position centerTopGoalFirst = { -7.9_in, 7.4_in };
    units::V2Position centerBottomGoalFirst = { -11_in, -11_in };

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    bool fast_wing = false;
    bool winging = true;
    Voltage slow_wing_speed = 0.5_volt;

    intake::in();
    intake::setColorSortEnabled(false);

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::up();

    RobotSetPose(-45.7, 15 * l, 0);
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // if (!bl) {
    //     LaserResets({
    //       &right_laser_model,
    //       &back_laser_model,
    //     });
    // }

    // std::cout << std::format("reset {:.2f},{:.2f},{:.2f}",
    //                          RobotGetPose().x.convert(in),
    //                          RobotGetPose().y.convert(in),
    //                          RobotGetPose().orientation.convert(deg))
    //           << std::endl;

    // only intake bottom balls to save time
    intake::setColorSortEnabled(false);
    // pros::delay(10);
    intake::set(intake::intake_bottom_balls);

    mb.moveTo(-24.3, 23.4 * l).drive_maxVolt(0.5_volt) | chain;

    mb.turnTo(-10, 36 * l) | chain;
    mb.moveTo(-10, 36 * l) | chain;

    mb.turnTo(-7.75, 39 * l) | chain;
    mb.boomerang(-7.75, 39 * l, bl ? 90 : 270).lead(0.3).timeout(0.5_sec) |
      chain;

    chain.waitUntil(closeEnough({ -24.3_in, 23.4_in * l }, 5_in));
    matchloader::down();
    pros::delay(200);
    matchloader::up();

    chain.wait();
    drivetrain.moveTank(0.2_volt, 0.2_volt);
    pros::delay(300);
    matchloader::up();

    // or
    mb.moveTo(-27.128, 31.982 * l).reverse() | chain;
    mb.moveTo(-38.472, long_goal * l)
        .reverse()
        .drive_kp(linear_pid.get_kp() * 0.8) |
      chain;
    chain.wait();

    mb.turnTo(-22_in, long_goal * l).reverse() | run;

    mb.moveTo(-26_in, (long_goal * l)).reverse() | chain;

    // mb.boomerang(-24_in, (match1 * l), 0)
    //     .reverse()
    //     .closeThreshold(30_in)
    //     .drive_minVolt(0.2_volt)
    //     .executeBeforeMotion([] {
    //         // controller.rumble(".");
    //     }) |
    //   chain;
    //
    // chain.waitUntil(closeEnough({ -26_in, long_goal * l }, 5_in));
    // intake::score_long();
    // pros::delay(1100);
    // chain.exitAll();

	chain.waitUntil(closeEnough({ -25_in, long_goal * l  }, 7.5_in));
	intake::score_long();
	pros::delay(300);
	chain.exitAll();

    mb.boomerang(-23_in, long_goal * l, 0)
        .reverse()
        .closeThreshold(100_in)
        .timeout(100_sec)
        .drive_toleranceDuration(100_sec)
        .drive_largeToleranceDuration(100_sec)
        .turn_kp(turn_drive_pid.get_kp() * 2)
        .turn_kd(turn_drive_pid.get_kd() * 0.5) |
      async;
    pros::delay(800);
    async.exitAll();

    matchloader::down();

    mb.turnTo(-80, (match1)*l) | chain;
    mb.moveTo(-60, (match1)*l)
        .drive_maxVolt(0.4_volt)
        // .timeout(1.3_sec)
        .executeBeforeMotion([] {
            pros::delay(400);
            intake::in();
        }) |
      chain;
    // chain.wait();

    chain.waitUntil(closeEnough({ -70_in, normal_match * l }, 14_in));
    pros::delay(300);
    chain.exitAll();

    mb.turnTo(-24_in, long_goal * l).reverse() | run;
    // mb.turnTo(0).reverse() | run;
    mb.moveTo(-26.5_in, (match1 * l))
        .reverse()
        .timeout(1.4_sec)
        .k_lat(0.0)

        .drive_maxVolt(0.65_volt)
        .closeThreshold(8_in)
        .executeAfterMotion([] {
            drivetrain.moveTank(-0.3_volt, -0.3_volt);
        }) |
      chain;

    chain.waitUntil(closeEnough({ -32_in, long_goal * l }, 4_in));
    intake::score_long();
    pros::delay(1300);
    matchloader::up();

    if (winging) {
        if (bl) {
            mb.moveTo(-35.737, 37.1) | chain;
            mb.turnTo(0).reverse() | chain;
            wings::down();
            mb.moveTo(-10.231, 37)
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
}

} // namespace elims_nineball_split
