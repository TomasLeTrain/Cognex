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

    LaserResets({
      &right_laser_model,
      &back_laser_model,
    });

    std::cout << std::format("reset {:.2f},{:.2f},{:.2f}",
                             RobotGetPose().x.convert(in),
                             RobotGetPose().y.convert(in),
                             RobotGetPose().orientation.convert(deg))
              << std::endl;

    // only intake bottom balls to save time
    intake::setColorSortEnabled(false);
    pros::delay(10);
    intake::in();

    if (bl) {
        mb.moveTo(-25.5, 23.4 * l).drive_maxVolt(0.3_volt) | chain;
        mb.moveTo(-28, 23.4 * l).drive_maxVolt(0.5_volt) | chain;
        //
        chain.waitUntil(closeEnough({ -25.5_in, 23.4_in * l }, 9.5_in));
        matchloader::down();

        chain.wait();
        //
        // pf_model.setDisabled(true);
        // mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y) | run;
        // pf_model.setDisabled(false);
        //
        // mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
        //     .k_lat(0.3)
        //     .drive_maxVolt(0.5_volt)
        //     .executeBeforeMotion([] {
        //         pros::Task([] {
        //             matchloader::down();
        //             pros::delay(200);
        //             matchloader::up();
        //         });
        //     }) |
        //   chain;

    } else {
        mb.moveTo(-28, 23.4 * l).drive_maxVolt(0.5_volt) | chain;
        mb.moveTo(-23.6, 23.6 * l) | async;

        chain.waitUntil(closeEnough({ -25.5_in, 23.4_in * l }, 9.5_in));
        matchloader::down();

        chain.wait();
        //
        // pf_model.setDisabled(true);
        // mb.turnTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y) | run;
        // pf_model.setDisabled(false);
        //
        // mb.moveTo(centerBottomGoalFirst.x, centerBottomGoalFirst.y)
        //     .k_lat(0.3)
        //     .drive_maxVolt(0.5_volt)
        //     .executeBeforeMotion([] {
        //         pros::Task([] {
        //             // pros::delay(200);
        //             matchloader::up();
        //         });
        //     }) |
        //   chain;
        //
        // chain.waitUntil(closeEnough({ -8_in, 8_in * l }, 5.5_in));
        // intake::score_bottom();
        // chain.wait();
        // pros::delay(1000);
    }

    mb.moveTo(-48, (match1)*l)
        // .reverse()
        // .only_y(true)
        .drive_maxVolt(0.5_volt)
        .reverse()
      // .drive_backwardsAccelSlew(0.1_volt)
      | run;

    // its a run here, so we can do these things
    intake::in();
    matchloader::down();
    // LaserResets({ &front_laser_model });

    mb.turnTo(-24_in, long_goal * l).reverse() | run;
    // mb.turnTo(0).reverse() | run;
    mb.moveTo(-26.5_in, (match1 * l))
        .reverse()
        .timeout(1.4_sec)
        .k_lat(0.0)

        // changed today!
        // .turn_kp(angular_pid.get_kp() * 0.5)
        // .turn_kd(angular_pid.get_kd() * 0.5)
        .drive_backwardsAccelSlew(0.4_volt)
        //

        .drive_maxVolt(0.6_volt)
        .closeThreshold(8_in)
        .executeAfterMotion([] {
            drivetrain.moveTank(-0.3_volt, -0.3_volt);
        }) |
      chain;

    chain.waitUntil(closeEnough({ -32_in, long_goal * l }, 4_in));
    intake::score_long();
    pros::delay(1300);

    mb.turnTo(-80, (match1)*l) | chain;
    mb.moveTo(-61.3, (match1)*l)
        .drive_maxVolt(0.65_volt)
        .timeout(1.8_sec)
        .executeBeforeMotion([] {
            pros::delay(400);
            intake::in();
        }) |
      chain;
    chain.wait();
    // pros::delay(100);

    mb.turnTo(-24_in, long_goal * l).reverse() | run;
    // mb.turnTo(0).reverse() | run;
    mb.moveTo(-26.5_in, (match1 * l))
        .reverse()
        .timeout(1.4_sec)
        .k_lat(0.0)

        // changed today!
        // .turn_kp(angular_pid.get_kp() * 0.5)
        // .turn_kd(angular_pid.get_kd() * 0.5)
        .drive_backwardsAccelSlew(0.4_volt)
        //

        .drive_maxVolt(0.65_volt)
        .closeThreshold(8_in)
        .executeAfterMotion([] {
            drivetrain.moveTank(-0.3_volt, -0.3_volt);
        }) |
      chain;

    chain.waitUntil(closeEnough({ -32_in, long_goal * l }, 4_in));
    intake::score_long();

    // pros::delay(500);
    // // start_time = now();
    //
    // intake::set(intake::scoring_long_top_balls_outake_bottom);
    //
    // pros::delay(1700);
}

} // namespace seven_ball
