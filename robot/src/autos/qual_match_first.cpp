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

    units::V2Position centerTopGoalFirst = { -7.9_in, 7.4_in };
    units::V2Position centerTopGoalSecond = { 1_in, 0_in };
    units::V2Position centerBottomGoalFirst = { 12_in, 12_in };
    units::V2Position centerBottomGoalSecond = { -12_in, -11_in };

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    double angle = 180;

    intake::setSkillsMiddleScoring(true);
    intake::setColorSortEnabled(false);

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(inactive);

    RobotSetPose(-48.2, 16.3, 90);
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    mb.moveTo(-48, match1)
        // .reverse()
        .only_y(true)
      // .drive_backwardsAccelSlew(0.1_volt)
      | run;
    // its a run here, so we can do these things
    intake::in();
    matchloader::down();

    mb.turnTo(-80, match1) | chain;
    mb.moveTo(-60.5, match1).drive_maxVolt(0.5_volt) | chain;
    chain.wait();
    pros::delay(100);

    mb.turnTo(-26.5_in, long_goal).reverse() | chain;
    mb.moveTo(-26.5_in, long_goal)
        .reverse()
        .timeout(1.3_sec)
        .k_lat(0.0)
        .drive_maxVolt(0.8_volt)
        .closeThreshold(8_in)
        .executeAfterMotion([] {
            drivetrain.moveTank(-0.3_volt, -0.3_volt);
        }) |
      chain;

    chain.waitUntil(closeEnough({ -32_in, -long_goal }, 4_in));
    intake::score_long();

	start_time = now();
	while()

    pros::delay(1000);
    // intake::in();

    // mb.moveTo(-31.5, 19.4).drive_maxVolt(0.3_volt) | run;
    // pros::delay(200);
    // intake::set(intake::intake_disabled);
    mb.moveTo(-25.5, 23.4) | async;

    pros::delay(300);

    // only intake bottom balls to save time
    pros::delay(10);
    intake::set(intake::intake_bottom_balls);

    async.waitUntil(closeEnough({ -23.758_in, 23.872_in }, 10_in));
    matchloader::down();

    async.wait();

    pf_model.setDisabled(true);
    mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | run;
    pf_model.setDisabled(false);

    mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
        .k_lat(0.3)
        .drive_maxVolt(0.5_volt)
        .executeBeforeMotion([] {
            pros::Task([] {
                matchloader::down();
                pros::delay(200);
                matchloader::up();
            });
        }) |
      chain;

    chain.waitUntil(closeEnough({ -8_in, 8_in }, 5.5_in));

    intake::out();
    pros::delay(150);
    intake::set(intake::scoring_middle_bottom_balls);
    chain.exitAll();
    drivetrain.moveTank(0.1_volt, 0.2_volt);
    // pros::delay(800);
}

} // namespace qual_match_first
