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
#include "pros/rtos.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/wings.h"
#include "units/Angle.hpp"
#include <iostream>

// do not do anything outside here!

namespace sunshine_elims {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

    auto closeEnough = [](units::V2Position target,
                          Length threshold) -> std::function<bool()> {
        return [target, threshold] -> bool {
            return RobotGetPose().distanceTo(target) < threshold;
        };
    };

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    Length match1 = normal_match;

    units::V2Position centerTopGoalFirst = { -8.3_in, 7.5_in };
    units::V2Position centerBottomGoalFirst = { -11_in, -11_in };

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    double angle = bl ? 0 : 0;

    // intake::setSkillsMiddleScoring(true);

    intake::in();

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::up();

    RobotSetPose(-47.2, 14.9 * l, angle);
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // only intake bottom balls to save time
    intake::setColorSortEnabled(false);
    intake::in();

    mb.moveTo(-25.5, 23.4 * l).drive_maxVolt(0.5_volt) | chain;
    mb.turnTo(-20.5, 23.4 * l) | chain;
    mb.moveTo(-20.5, 23.4 * l).drive_maxVolt(0.5_volt) | chain;

    chain.waitUntil(closeEnough({ -23.758_in, 23.872_in * l }, 11.0_in));

    matchloader::down();
    pros::delay(500);
    matchloader::up();

    chain.wait();
    pros::delay(200);
    // intake::set(intake::intake_top_balls);
    // intake::set(intake::intake_bottom_balls);
	intake::in();

    // mb.turnTo(-10.763, 42.455 * l) | chain;
    // mb.moveTo(-10.763, 42.455 * l)
    //     .drive_maxVolt(0.3_volt)
    //     .executeBeforeMotion([] {
    //         pros::delay(140);
    //         matchloader::down();
    //         // intake::in();
    //         intake::set(intake::intake_bottom_balls);
    //     }) |
    //   chain;
    //
    // chain.waitUntil(closeEnough({ -6.763_in, 42.555_in * l }, 5_in));
    // matchloader::down();

    // chain.wait();
    // pros::delay(200);
    // matchloader::up();

    mb.moveTo(-17, 20 * l).reverse() | chain;
    mb.moveTo(-30.413, 20 * l).reverse() | chain;

    // mb.moveTo(-29.46, 35.257 * l).reverse() | chain;
    mb.turnTo(-36.687, 47.2 * l) | chain;
    mb.moveTo(-36.687, 47.2 * l).only_y(true) | chain;
    chain.wait();

    mb.turnTo(-24, long_goal * l).reverse() | run;
    mb.moveTo(-26_in, (long_goal * l))
        .reverse()
        .timeout(1.4_sec)
        .k_lat(0.0)

        // changed today!
        // .turn_kp(angular_pid.get_kp() * 0.5)
        // .turn_kd(angular_pid.get_kd() * 0.5)
        .drive_backwardsAccelSlew(0.4_volt)
        //

        .drive_maxVolt(0.5_volt)
        .closeThreshold(8_in)
        .executeAfterMotion([] {
            // drivetrain.moveTank(-0.3_volt, -0.3_volt);

            mb.arc(180, -1.0)
                .timeout(100_sec)
                .turn_errorTolerance(0_stDeg)
                .turn_largeErrorTolerance(0_stDeg)
                .turn_toleranceDuration(100_sec)
                .turn_largeToleranceDuration(100_sec) |
              async;
            // drivetrain.moveTank(-0.3_volt, -0.0_volt);
        }) |
      chain;

    chain.waitUntil(closeEnough({ -32_in, long_goal * l }, 4_in));
    intake::score_long();

    // pros::delay(500);
    // // start_time = now();
    //
    // intake::set(intake::scoring_long_top_balls_outake_bottom);

    pros::delay(500);
    matchloader::down();
    pros::delay(1000);
    async.exitAll();

    mb.turnTo(-80, match1 * l) | chain;
    mb.moveTo(-61, match1 * l).drive_maxVolt(0.6_volt) | chain;
    pros::delay(250);
    intake::in();
    chain.wait();
    pros::delay(200);

    if (bl) {
        // mb.moveTo(-30, 30 * l).reverse().executeAfterMotion([] {
        //     intake::set(intake::outtake_bottom_balls);
        // }) |
        //   chain;
        // mb.turnTo(-26, 22 * l) | chain;
        // chain.wait();
        mb.moveTo(-26, 22.5 * l)
            .reverse()
            .drive_maxVolt(0.5_volt)
            .executeAfterMotion([] {
                pros::Task([] {
                    pros::delay(100);
                    intake::set(intake::outtake_bottom_balls);
                });
            })
          // chain;
          | run;

        pf_model.setDisabled(true);
        mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | run;
        pf_model.setDisabled(false);

        mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
            .k_lat(0.3)
            .drive_maxVolt(0.3_volt)
            .executeBeforeMotion([] {
                pros::Task([] {
                    // matchloader::down();
                    pros::delay(300);
                    matchloader::up();
                });
            }) |
          chain;

        chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 5.5_in));

        // intake::out();
        // pros::delay(230);
        intake::set(intake::scoring_middle_top_balls);
        chain.exitAll();
        drivetrain.moveTank(0.05_volt, 0.05_volt);

        pros::delay(1000);
        intake::set(intake::scoring_middle_bottom_balls);
    }
}

} // namespace sunshine_elims
