#include "apis.h"
//
#include "autos.h"
#include "globals/blazing_globals.h"
#include "pros/abstract_motor.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"

// do not do anything outside here!

namespace roboticon_skills {

// you can add any variables / functions here

void run_auton() {
    // do whatever you want here
    // changePoseGetter(&smoother_model);
    // pros::delay(30);

    Time start_time = now();

    // RobotSetPose(0, 0, 90);
    //
    // mb.moveTo(0_in, 2_tile) | run;
    // std::cout << tracker.getPosition().x.convert(in) << " "
    //           << tracker.getPosition().y.convert(in) << " "
    //           << tracker.getAngle().convert(deg) << std::endl;
    //
    // mb.turnTo(180) | run;
    //
    // std::cout << tracker.getPosition().x.convert(in) << " "
    //           << tracker.getPosition().y.convert(in) << " "
    //           << tracker.getAngle().convert(deg) << std::endl;
    //
    // pros::delay(10000);
    //
    // return;

    RobotSetPose(63.118, -15.625, 90);

    if (false) {
        RobotSetPose(-63.118, -15.625, 90);

        // drivetrain.setBrakeMode(pros::MotorBrake::hold);

        pros::delay(50);

        intake::set(intake::intake);

        // pull matchloader down
        mb.moveTo(-63, 18) | chain;

        // go towards top left ball cluster
        mb.boomerang(-32, 31.7, 315)
            .lead(0.35, 0.1)
            .linear_clampMaxVoltage(0.6_volt) |
          chain;

        size_t top_left_cluster = chain.getCurrentIndex();

        // go to top center
        // mb.boomerang(-13, 12, 315) | chain;
        mb.moveTo(-13, 13.5)
            // make large timeout not affect as much
            .largeLinearToleranceDuration(2_sec)
            // lower the error tolerance
            .linearErrorTolerance(2_in) |
          chain;

        // pros::delay(1000);
        // pull matchloader up

        // wait for boomerang to finish
        chain.waitUntilIndex(top_left_cluster);

        // finished the boomerang, pull matchloader down

        // wait to get to goal
        chain.wait();
        std::cout << "position after move to middle: "
                  << tracker.getPosition().x.convert(in) << " "
                  << tracker.getPosition().y.convert(in) << " "
                  << tracker.getAngle().convert(deg) << std::endl;

        // while (true) {
        //     pros::lcd::print(0,
        //                      "%f %f %f",
        //                      to_in(arc_pose_tracker.getPosition().x),
        //                      to_in(arc_pose_tracker.getPosition().y),
        //                      to_stDeg(arc_pose_tracker.getAngle()));
        //     pros::delay(10);
        // }

        // score top center

        // back up and go to bottom right cluster
        mb.turnTo(270_stDeg)
            .direction(AngularDirection::RIGHT)
            .radius(-1.0) // makes it a swing
            .chainAngularErrorTolerance(10_stDeg)
            .linear_decelSlew(0.1_volt)
            .setChainTime(100_msec) // instant switch
          | chain;

        // bottom-left middle ball cluster
        mb.boomerang(-22, -22, 260) | chain;

        size_t bottom_left_cluster_index = chain.getCurrentIndex();

        // go to matchloader
        mb.boomerang(-52.4_in, -2_tile, 180_stDeg)
            // .lead(0.4, 0.15)
            .lead(0.4)
            .linear_clampMaxVoltage(0.5_volt) |
          chain;

        // the matchloader is already pulled down at this point,
        // don't have to worry about it
        // mb.boomerang(-57_in, -2_tile, 180_stDeg) | chain;
        mb.moveTo(-57_in, -2_tile) | chain;

        // waits until gets to cluster to pull matchloader down
        chain.waitUntilIndex(bottom_left_cluster_index);

        // pull matchloader down

        // wait until all queued motions stop
        chain.wait();
        std::cout << "position at match: "
                  << tracker.getPosition().x.convert(in) << " "
                  << tracker.getPosition().y.convert(in) << " "
                  << tracker.getAngle().convert(deg) << std::endl;

        // pros::delay(1000);

        mb.moveTo(-30.8, -47.1).reverse() | run;
        //
        // score on long goal
        //

        // turn around and go towards matchloader
        left_motors.set_brake_mode(pros::MotorBrake::brake);
        mb.turnTo(0_stDeg)
            .direction(AngularDirection::LEFT)
            .radius(1.0) // makes it a swing
            .chainAngularErrorTolerance(10_stDeg)
            .setChainTime(0_msec) // instant switch
          | chain;
        size_t bottom_swing_to_other_side = chain.getCurrentIndex();

        // go to other side of the field, close to the wall
        mb.moveTo(22.41, -60) | chain;

        // go to matchloader
        mb.boomerang(52.4_in, -2_tile, 0_stDeg).lead(0.5) | chain;
        size_t bottom_right_matchloader = chain.getCurrentIndex();
        mb.moveTo(57_in, -2_tile) | chain;

        chain.waitUntilIndex(bottom_swing_to_other_side);
        drivetrain.setBrakeMode(pros::MotorBrake::coast);

        chain.waitUntilIndex(bottom_right_matchloader);
        // set matchloader down

        chain.wait();

        // matchload

        mb.moveTo(30.8, -47.1).reverse() | run;
        //
        // score
        //

        chain.wait();
        std::cout << "final pos: " << tracker.getPosition().x.convert(in) << " "
                  << tracker.getPosition().y.convert(in) << " "
                  << tracker.getAngle().convert(deg) << std::endl;
        while (true) {
            pros::delay(10);
        }

        mb.boomerang(62.2, -17, 90) | chain;

        chain.wait();
    }

    // matchload down?
    mb.moveTo(63.4, 21) | chain;

    mb.boomerang(30.5, 27, 230)
        .lead(0.35, 0.1)
        .linear_clampMaxVoltage(0.6_volt) |
      chain;
    std::cout << "added boomerang" << std::endl;

    size_t top_right_cluster = chain.getCurrentIndex();

    mb.moveTo(14, 15) | chain;

    std::cout << "waiting" << std::endl;
    chain.waitUntilIndex(top_right_cluster);
    std::cout << "waited" << std::endl;

    // matchload down
    //
    // wait a bit, matchload up
    // pros::delay(100);

    // wait for all motions to complete
    chain.wait();
    std::cout << "all finished" << std::endl;

    // score bottom

    // back up
    mb.moveTo(42_in, 2_tile).reverse() | chain;
    mb.boomerang(52.4_in, 2_tile, 0_stDeg).linear_clampMaxVoltage(0.6_volt) | chain;
    size_t top_right_matchloader = chain.getCurrentIndex();

    mb.moveTo(57_in, 2_tile) | chain;

    chain.waitUntilIndex(top_right_matchloader);
    // pull matchloader down

    chain.wait();
    // get matchloader
    // pros::delay(1000);

    mb.moveTo(30.8_in, 2_tile).reverse() | run;

    // go score on long goal, pull matchloader up
    // pros::delay(1000);

    // go to other side of long goal and matchloader
    left_motors.set_brake_mode(pros::MotorBrake::brake);
    mb.turnTo(180_stDeg)
        .direction(AngularDirection::LEFT)
        .radius(1.0) // makes it a swing
        .chainAngularErrorTolerance(10_stDeg)
        .setChainTime(0_msec) // instant switch
      | chain;
    size_t top_swing_to_other_side = chain.getCurrentIndex();

    // go to other side of the field, close to the wall
    mb.moveTo(-22.41, 60) | chain;

    // go to matchloader
    mb.boomerang(-52.4, 46.7, 180) | chain;
    size_t top_left_matchloader = chain.getCurrentIndex();

    mb.moveTo(-57, 46.7) | chain;

    chain.waitUntilIndex(top_swing_to_other_side);
    drivetrain.setBrakeMode(pros::MotorBrake::coast);

    chain.waitUntilIndex(top_left_matchloader);
    // pull matchloader down

    chain.wait();

    // matchload

    mb.moveTo(-30.8, 47.1).reverse() | run;
    //
    // score
    //

    // finish, go to park :)
    mb.boomerang(-63, 24.5, 270) | chain;

    chain.wait();
    // mb.moveTo(-63, 0) | chain;
    //
    // chain.wait();

    std::cout << "finished run in time: " << now() - start_time << std::endl;

    pros::delay(30);
}

} // namespace roboticon_skills
