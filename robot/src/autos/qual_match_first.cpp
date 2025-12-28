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

    auto closeEnough = [](units::V2Position target,
                          Length threshold) -> std::function<bool()> {
        return [target, threshold] -> bool {
            return RobotGetPose().distanceTo(target) < threshold;
        };
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

    units::V2Position centerTopGoalFirst = { -8.55_in, 7.5_in };
    units::V2Position centerTopGoalSecond = { 1_in, 0_in };
    units::V2Position centerBottomGoalFirst = { 12_in, 12_in };
    units::V2Position centerBottomGoalSecond = { -12_in, -11_in };

    alliance_t opposite_alliance =
      auto_alliance == alliance_t::red ? alliance_t::blue : alliance_t::red;

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    double angle = bl ? 90 : 270;

    intake::setSkillsMiddleScoring(true);
    intake::setColorSortEnabled(false);

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(inactive);

    pf_model.setDisabled(true);

    RobotSetPose(-48.2, 16.3 * l, angle);
    LaserResets({
      &left_laser_model,
      &front_laser_model,
    });

    setSmootherAlphas(smoother_config.alpha_x * 0.0,
                      smoother_config.alpha_y * 0.0);

    std::cout << std::format("reset {:.2f},{:.2f},{:.2f}",
                             RobotGetPose().x.convert(in),
                             RobotGetPose().y.convert(in),
                             RobotGetPose().orientation.convert(deg))
              << std::endl;

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    mb.moveTo(-48, (match1 - 0.75_in) * l)
        // .reverse()
        .only_y(true)
        .drive_maxVolt(0.5_volt)
      // .drive_backwardsAccelSlew(0.1_volt)
      | run;

    std::cout << std::format("moveto1 {:.2f},{:.2f},{:.2f}",
                             RobotGetPose().x.convert(in),
                             RobotGetPose().y.convert(in),
                             RobotGetPose().orientation.convert(deg))
              << std::endl;

    // its a run here, so we can do these things
    intake::in();
    matchloader::down();
    // LaserResets({ &front_laser_model });

    // mb.turnTo(-80, match1*l) | chain;
    mb.turnTo(180) | run;
    mb.distanceAtHeading(15_in)
        // mb.moveTo(-60.5, (match1)*l)
        //     .drive_maxVolt(0.9_volt)
        .timeout(2.0_sec)
      // .executeAfterMotion([] {
      //     drivetrain.moveTank(-0.1_volt, -0.1_volt);
      // }) |
      | run;

    std::cout << std::format("after chain {:.2f},{:.2f},{:.2f}",
                             RobotGetPose().x.convert(in),
                             RobotGetPose().y.convert(in),
                             RobotGetPose().orientation.convert(deg))
              << std::endl;

    // pros::delay(100);

    mb.turnTo(-24_in, long_goal * l).reverse() | run;
    // mb.turnTo(0).reverse() | run;
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
            drivetrain.moveTank(-0.3_volt, -0.3_volt);
        }) |
      chain;

    chain.waitUntil(closeEnough({ -32_in, long_goal * l }, 4_in));
    intake::score_long();

    pros::delay(500);
    // start_time = now();

    intake::set(intake::scoring_long_top_balls_outake_bottom);

    pros::delay(1700);

    // bool timeout = timeoutDone(2_sec, start_time);
    // bool detect_opposite =
    //   intake::getMiddleDetectedColor() == opposite_alliance;
    // while (true) {
    //     timeout = timeoutDone(2_sec, start_time);
    //     detect_opposite = intake::getMiddleDetectedColor() ==
    //     opposite_alliance; if (timeout || detect_opposite) break;
    //     pros::delay(10);
    // }

    // if (detect_opposite) {
    //     // pros::delay(150);
    //     intake::set(intake::scoring_long_top_balls_outake_bottom);
    //
    //     pros::delay(static_cast<int>(to_msec(now() - start_time)));
    //     // pros::delay(500);
    // } else {
    // }

    // intake::in();

    // mb.moveTo(-31.5, 19.4).drive_maxVolt(0.3_volt) | run;
    // pros::delay(200);
    // intake::set(intake::intake_disabled);

    matchloader::up();

    mb.moveTo(-26, 23.4 * l) | async;

    pros::delay(600);

    // only intake bottom balls to save time
    pros::delay(10);
    intake::set(intake::intake_bottom_balls);

    async.waitUntil(closeEnough({ -23.758_in, 23.872_in * l }, 10.5_in));
    matchloader::down();

    async.wait();

    if (bl) {
        pf_model.setDisabled(true);
        mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | run;
        pf_model.setDisabled(false);

        mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
            .k_lat(0.3)
            .drive_maxVolt(0.35_volt)
            .executeBeforeMotion([] {
                pros::Task([] {
                    // matchloader::down();
                    pros::delay(300);
                    matchloader::up();
                });
            }) |
          chain;

        // chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 12_in));
        // matchloader::up();
        chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 5.5_in));

        intake::out();
        pros::delay(230);
        intake::set(intake::scoring_middle_bottom_balls);
        chain.exitAll();
        drivetrain.moveTank(0.05_volt, 0.05_volt);
        // pros::delay(800);
    } else {
        pf_model.setDisabled(true);
        mb.turnTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y) | run;
        pf_model.setDisabled(false);

        mb.moveTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y)
            .k_lat(0.3)
            .drive_maxVolt(0.35_volt)
            .executeBeforeMotion([] {
                pros::Task([] {
                    // matchloader::down();
                    pros::delay(300);
                    matchloader::up();
                });
            }) |
          chain;

        // chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 12_in));
        // matchloader::up();
        chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 5.5_in));

        intake::out();
        pros::delay(230);
        intake::set(intake::scoring_middle_bottom_balls);
        chain.exitAll();
        drivetrain.moveTank(0.05_volt, 0.05_volt);
        // pros::delay(800);
    }
}

} // namespace qual_match_first
