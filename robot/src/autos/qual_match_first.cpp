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
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/wings.h"
#include "units/Angle.hpp"
#include <iostream>

// do not do anything outside here!

namespace qual_match_first {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here

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

    units::V2Position centerTopGoalFirst = { -7.9_in, 7.4_in };
    // units::V2Position centerBottomGoalFirst = { -10.3_in, -11_in };
    units::V2Position centerBottomGoalFirst = { -12.4_in, -13.4_in };

    alliance_t opposite_alliance =
      auto_alliance == alliance_t::red ? alliance_t::blue : alliance_t::red;

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    double angle = bl ? 90 : 270;

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
            .turn_kp(turn_drive_pid.get_kp() * 2)
            .turn_kd(turn_drive_pid.get_kd() * 0.5) |
          async;
        pros::delay(to_msec(score_time - 300_msec));
        async.exitAll();
    };

    intake::setSkillsMiddleScoring(true);
    intake::setColorSortEnabled(false);

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(inactive);

    RobotSetPose(-48.2, 16.3 * l, angle);

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    mb.moveTo(-48, match1 * l)
        // .reverse()
        .only_y(true)
        .drive_maxVolt(0.6_volt)
      // .drive_backwardsAccelSlew(0.1_volt)
      | run;

    // its a run here, so we can do these things
    intake::in();
    matchloader::down();

    matchload(-1,
              l,
              1_sec,
              500_msec,
              7.5_in,
              linear_pid.get_kp() * 0.7,
              linear_pid.get_kd() * 0.5,
              1.0_volt);

    score_long_goal(-1, l, 1.0_sec);

    matchloader::up();

    // mb.moveTo(-23.4, 23.4 * l).executeAfterMotion([] {
    //     pros::Task([] {
    //         pros::delay(250);
    //         intake::set(intake::intake_bottom_balls);
    //     });
    // }) |
    //   async;
    //
    // async.waitUntil(closeEnough({ -23.4_in, 23.4_in * l }, 10.5_in));
    // matchloader::down();
    // async.wait();

    // only intake bottom balls to save time

    // if (bl) {
    //     mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | run;
    //
    //     mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
    //         .k_lat(0.3)
    //         .drive_maxVolt(0.35_volt)
    //         .executeBeforeMotion([] {
    //             pros::Task([] {
    //                 // matchloader::down();
    //                 pros::delay(300);
    //                 matchloader::up();
    //             });
    //         }) |
    //       chain;
    //
    //     // chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 12_in));
    //     // matchloader::up();
    //     chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 5.5_in));
    //
    //     intake::out();
    //     pros::delay(230);
    //     intake::set(intake::scoring_middle_bottom_balls);
    //     chain.exitAll();
    //     drivetrain.moveTank(0.05_volt, 0.05_volt);
    //     // pros::delay(800);
    // } else {
    //     pf_model.setDisabled(true);
    //     mb.turnTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y) | run;
    //     pf_model.setDisabled(false);
    //
    //     mb.moveTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y)
    //         .k_lat(0.3)
    //         .drive_maxVolt(0.35_volt)
    //         .executeBeforeMotion([] {
    //             pros::Task([] {
    //                 // matchloader::down();
    //                 pros::delay(300);
    //                 matchloader::up();
    //             });
    //         }) |
    //       chain;
    //
    //     // chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 12_in));
    //     // matchloader::up();
    //     chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 5.5_in));
    //
    //     intake::out();
    //     pros::delay(230);
    //     intake::set(intake::scoring_middle_bottom_balls);
    //     chain.exitAll();
    //     drivetrain.moveTank(0.05_volt, 0.05_volt);
    //     // pros::delay(800);
    // }

    if (bl) {
        mb.moveTo(-25.5, 23.4 * l)
            .executeBeforeMotion([] {
                pros::Task([] {
                    pros::delay(200);
                    intake::set(intake::intake_bottom_balls);
                });
            })
            .drive_maxVolt(0.5_volt) |
          async;

        async.waitUntil(closeEnough({ -25.5_in, 23.4_in * l }, 10_in));
        matchloader::down();

        async.wait();

        mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | chain;

        mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
            .k_lat(0.3)
            .drive_maxVolt(0.5_volt)
            .executeBeforeMotion([] {
                // pros::Task([] {
                //     // matchloader::down();
                //     // pros::delay(200);
                //     // matchloader::up();
                // });
            }) |
          chain;

        chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 5.5_in));
        intake::set(intake::outtake_open_middle);
        pros::delay(100);
        intake::set(intake::scoring_middle_bottom_balls_slow);
        pros::delay(1500);
        chain.exitAll();
    } else {
        mb.moveTo(-23.6, 23.6 * l)
            .executeBeforeMotion([] {
                pros::Task([] {
                    pros::delay(200);
                    intake::set(intake::intake_bottom_balls);
                });
            })
            .drive_maxVolt(0.5_volt) |
          async;

        async.waitUntil(closeEnough({ -23.6_in, 23.6_in * l }, 10_in));
        matchloader::down();

        async.wait();

        mb.turnTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y) | run;

        mb.moveTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y)
            .k_lat(0.3)
            .drive_maxVolt(0.7_volt)
            .executeBeforeMotion([] {
                pros::Task([] {
                    // pros::delay(200);
                    matchloader::up();
                });
            }) |
          chain;
        mb.turnTo(-8, -8) | chain;

        chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 5.5_in));
        intake::set(intake::score_bottom_slow);
        // intake::set(intake::score_bottom_bottom_balls_slow);
        chain.wait();

        while (true) {
            drivetrain.moveTank(0.15_volt, 0.15_volt);
            pros::delay(200);
            drivetrain.moveTank(-0.15_volt, -0.15_volt);
            pros::delay(200);
        }
    }
}

} // namespace qual_match_first
