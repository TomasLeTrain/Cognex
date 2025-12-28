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

namespace sunshine_quals {

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

    units::V2Position centerTopGoalFirst = { -7.9_in, 7.4_in };
    units::V2Position centerBottomGoalFirst = { -11_in, -11_in };

    bool bl =
      (auto_side == field_side_t::left || auto_side == field_side_t::unset);

    int l = bl ? 1 : -1;

    double angle = bl ? 0 : 0;

    intake::setSkillsMiddleScoring(true);

    intake::in();

    /* START AUTON */

    // pull wing up to avoid any collision with game objects (bad for cog?)
    wings::set(inactive);

    RobotSetPose(-47.2, 14.9 * l, angle);
    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    // only intake bottom balls to save time
    intake::setColorSortEnabled(false);
    pros::delay(10);
    intake::set(intake::intake_bottom_balls);

    if (bl) {
        mb.moveTo(-25.5, 23.4 * l) | async;

        async.waitUntil(closeEnough({ -31.93_in, 18.784_in * l }, 5_in));
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

    } else {
        mb.moveTo(-23.6, 23.6 * l) | async;

        async.waitUntil(closeEnough({ -31.93_in, 18.784_in * l }, 5_in));
        matchloader::down();

        async.wait();

        pf_model.setDisabled(true);
        mb.turnTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y) | run;
        pf_model.setDisabled(false);

        mb.moveTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y)
            .k_lat(0.3)
            .drive_maxVolt(0.5_volt)
            .executeBeforeMotion([] {
                pros::Task([] {
                    // pros::delay(200);
                    matchloader::up();
                });
            }) |
          chain;

        chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 5.5_in));
        intake::score_bottom();
        chain.wait();
        pros::delay(1000);
    }

    mb.moveTo(-48, match1 * l)
        .reverse()
        // .only_y(true)
        .drive_backwardsAccelSlew(0.1_volt) |
      run;
    // its a run here, so we can do these things
    intake::in();
    intake::setColorSortEnabled(false);
    matchloader::down();

    mb.turnTo(-80, match1 * l) | chain;
    mb.moveTo(-61, match1 * l).drive_maxVolt(0.8_volt) | chain;
    chain.wait();
    pros::delay(200);

    mb.turnTo(-26.5_in, long_goal * l).reverse() | chain;
    mb.moveTo(-26.5_in, long_goal * l)
        .reverse()
        .timeout(1.3_sec)
        .k_lat(0.0)
        .drive_maxVolt(0.8_volt)
        .closeThreshold(8_in)
        .executeAfterMotion([] {
            drivetrain.moveTank(-0.3_volt, -0.3_volt);
        }) |
      chain;

    chain.waitUntil(closeEnough({ -32_in, -long_goal * l }, 4_in));
    intake::score_long();
    pros::delay(1000);
    intake::in();
}

} // namespace sunshine_quals
