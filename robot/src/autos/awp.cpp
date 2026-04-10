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
#include "globals/config.h"
#include "globals/device_globals.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include <iostream>
#include <tuple>

namespace awp {

void pre_auton() {
    // set the robot state to match expectations
    // done in case driver or such is run before auto
    wings::up();
    odom_retract::lowerOdom();
    matchloader::up();

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    intake::setAutonColorSort(false);
}

void matchload(double sign_x, double sign_y, Time matchload_time) {
    auto make_matchloader_point = [](double sign_x,
                                     double sign_y) -> units::V2Position {
        Length normal_match = 46.7_in;
        return { 67.4_in * sign_x, normal_match * sign_y };
    };

    auto make_machloader_pose = [](units::V2FPosition target,
                                   Length distance) -> units::Pose {
        const auto error_unit_vector = (RobotGetPose() - target).normalize();
        auto final_point = target + distance * error_unit_vector;

        return units::Pose { final_point, final_point.angleTo(target) };
    };

    units::V2Position target_Point = make_matchloader_point(sign_x, sign_y);

    // 11 is barely achievable - 0.1 less than achievable
    Length target_dist = 10.9_in;

    auto func = [&] -> units::Pose {
        return make_machloader_pose(target_Point, target_dist);
    };

    // make sure we are matchloading
    matchloader::down();

    mb.turnTo(func())
        .turn_toleranceDuration(0_sec)
        .turn_errorTolerance(4.0_stDeg)
        .turn_velocityTolerance(400_radps) |
      run;

    mb.moveTo(func())
        // if it takes longer it most likely got stuck
        .timeout(1.5_sec)
        .drive_vel_accelSlew(110_inps2)
        .drive_toleranceDuration(100_sec)
        .drive_largeToleranceDuration(100_sec)
        .drive_vel_mp_setMaxAccel(70_inps2) |
      async;

    Length matchload_start_distance = 13_in;

    auto custom_exit_condition = [&] -> bool {
        const auto curr_pose = RobotGetPose();
        const auto error = (target_Point - curr_pose);
        const auto [forwards_error, sideways_error] =
          error.rotatedBy(-curr_pose.orientation);

        const bool close = error.magnitude() <
                           // trigger only if closes to the matchloader
                           matchload_start_distance + 5_in;

        const bool forwards_close =
          units::abs(forwards_error) < matchload_start_distance;

        // use forwards error and
        return close && forwards_close;
    };

    auto wait_result = async.waitOr(custom_exit_condition, 3_sec);

    if (wait_result == AsyncExecutorBase::motionFinished ||
        wait_result == AsyncExecutorBase::timeoutFinished) {
        // custom condition did not trigger, meaning we got stuck or
        // something else went wrong. Don't wait just exit
        async.exitAll();
    } else {
        // got to matcloader successfully, start matchloading
        async.exitAll();
        // no motions should be executing here, so setting the voltage instantly
        // should be fine

        // passive voltage forwards since motion might oscilate
        drivetrain.moveTank(0.2_volt, 0.2_volt);
        pros::delay(to_msec(matchload_time));
    }
}

void score_long_goal(double sign_x, double sign_y, Time score_time) {
    // turn to goal, reversed
    Length long_goal = 47.0_in;

    mb.moveTo(30_in * sign_x, long_goal * sign_y).reverse() | run;

    // auto target_backwards_heading = sign_x == -1 ? 0_stDeg : 180_stDeg;
    // auto target_forwards_heading = sign_x == -1 ? 180_stDeg : 0_stDeg;
    // auto boomerang_heading = sign_x == -1 ? 170_stDeg : 350_stDeg;

    // units::Pose target_pose = { 24_in * sign_x,
    //                             long_goal * sign_y,
    //                             target_backwards_heading };
    //
    // auto exit_condition = [&] -> bool {
    //     auto curr_pose = RobotGetPose();
    //     bool x_close =
    //       units::abs(curr_pose.x) >= 27_in && units::abs(curr_pose.x) <=
    //       40_in;
    //     // bool y_close =
    //     //   units::abs(curr_pose.y) >= 43_in && units::abs(curr_pose.y) <=
    //     //   51_in;
    //     // //
    //     // bool theta_close =
    //     //   units::abs(angleError(target_forwards_heading,
    //     //                         curr_pose.orientation)) <= 25_stDeg;
    //
    //     // return x_close && y_close && theta_close;
    //     return x_close;
    // };
    //
    // mb.moveTo(target_pose)
    //     // .drive_vel_mp_setMaxAccel(110_inps2)
    //     .only_x(true, 28_in * sign_x)
    //     .closeThreshold(7_in)
    //     // .turn_kp(13.9)
    //     .timeout(2_sec)
    //     .reverse() |
    //   chain;
    //
    // chain.waitUntil(exit_condition);
    //
    // // regardless of getting stuck or not we perform the same action
    // intake::score_long();
    //
    // pros::delay(200);
    //
    // // exit regardless to have better aligner
    // chain.exitAll();
    //
    // pros::delay(10);
    //
    // // queue aligning motion
    // mb.turnTo(target_forwards_heading)
    //     .turn_toleranceDuration(100_sec)
    //     .turn_largeToleranceDuration(100_sec)
    //     .timeout(0.3_sec)
    //     .radius(-10.5_in / 2) |
    //   async;
    // mb.turnTo(target_forwards_heading)
    //     .turn_toleranceDuration(100_sec)
    //     .turn_largeToleranceDuration(100_sec)
    //     .constantVelocity(-15_inps) |
    //   async;
    // pros::delay(to_msec(score_time));
    //
    // async.exitAll();
}

std::shared_ptr<lyfast::geometry::Line>
line(float x0, float y0, float x1, float y1) {
    return std::make_shared<lyfast::geometry::Line>(
      units::V2FPosition { from_in(x0), from_in(y0) },
      units::V2FPosition { from_in(x1), from_in(y1) });
}

std::shared_ptr<lyfast::geometry::CubicBezier> curve(float x0,
                                                     float y0,
                                                     float x1,
                                                     float y1,
                                                     float x2,
                                                     float y2,
                                                     float x3,
                                                     float y3) {
    return std::make_shared<lyfast::geometry::CubicBezier>(
      units::V2FPosition { from_in(x0), from_in(y0) },
      units::V2FPosition { from_in(x1), from_in(y1) },
      units::V2FPosition { from_in(x2), from_in(y2) },
      units::V2FPosition { from_in(x3), from_in(y3) });
}

namespace skills_paths {
auto start_TO_in_red_park = line(-44.125, 0.039, -61.257, 0.363);
auto in_red_park_TO_out_of_red = line(-61.257, 0.363, -44.365, 0.235);
auto out_of_red_TO_get_blue_middle =
  curve(-44.365, 0.235, -30.572, -0.922, -16.487, 8.767, -17.78, 17.656);
auto get_blue_middle_TO_score_middle = line(-17.78, 17.656, -13.179, 12.513);
auto score_middle_TO_ull =
  curve(-13.179, 12.513, -30.308, 27.086, -32.253, 47.2, -37.891, 47.2);
auto ull_TO_uls = line(-37.891, 47.2, -30.06, 47.2);
auto uls_TO_ulm = line(-30.06, 47.2, -57.173, 46.6);
auto ulm_TO_url1 =
  curve(-57.173, 46.6, -37.078, 46.6, -49.792, 67.874, 22.979, 59.339);
auto url1_TO_End_Control =
  curve(22.979, 59.339, 30.56, 58.034, 34.696, 53.895, 36, 47.2);
auto End_Control_TO_urls = line(36, 47.2, 30.2, 47.2);
auto urls_TO_urm = line(30.2, 47.2, 56.959, 46.6);
auto urm_TO_urls2 = line(56.959, 46.6, 30.2, 47.2);
auto urls2_TO_ur_cluster =
  curve(30.2, 47.2, 40.083, 46.256, 27.22, 37.467, 30.776, 31.164);
auto ur_cluster_TO_blue_park = line(30.776, 31.164, 44.859, -0.234);
auto blue_park_TO_in_blue_park = line(44.859, -0.234, 61.945, -0.234);
auto in_blue_park_TO_blue_park2 = line(61.945, -0.234, 45.304, -0.056);
auto blue_park2_TO_go_bottom = line(45.304, -0.056, 16.95, 18.346);
auto go_bottom_TO_bottom_score = line(16.95, 18.346, 11.967, 12.295);
auto bottom_score_TO_back_bottom = line(11.967, 12.295, 16.594, 16.389);
auto back_bottom_TO_dr_cluster = line(16.594, 16.389, 23.741, -23.427);
auto dr_cluster_TO_drl =
  curve(23.741, -23.427, 36.435, -44.03, 35.082, -47.2, 39.523, -47.2);
auto drl_TO_drls = line(39.523, -47.2, 29.949, -47.2);
auto drls_TO_drm = line(29.949, -47.2, 57.098, -47.059);
auto drm_TO_dll =
  curve(57.098, -47.059, 25.566, -46.6, 48.236, -64.96, -22.032, -60.53);
auto dll_TO_dls =
  curve(-22.032, -60.53, -37.44, -60.465, -45.449, -47.829, -31.435, -47.2);
auto dls_TO_dlm = line(-31.435, -47.2, -56.502, -47.295);
auto dlm_TO_dls2 = line(-56.502, -47.295, -31.435, -47.2);
auto dls2_TO_ending =
  curve(-31.435, -47.2, -66.497, -47.2, -61.239, -20.894, -62.307, -0.693);
} // namespace skills_paths

void silly() {
    using namespace blazing::lyfast;
    using namespace blazing::lyfast::geometry;
    using namespace blazing::lyfast::mp;

    std::shared_ptr<geometry::Spline> spline_ptr { new Spline(
      { skills_paths::ulm_TO_url1, skills_paths::url1_TO_End_Control }) };

    // auto start_position = spline_ptr->getFirstEndpoint();
    // auto start_angle = spline_ptr->df(0).getAngle();
    // arc_pose_tracker.setPose({ start_position, start_angle });

    RobotConstraints robot_constraints(
      10.5_in, // track with
      // 0.05, // friction coeff - should tune?
      1.00, // friction coeff - should tune?
      3.25_in, // wheel diameter
      389_rpm, // max ang vel - determined somewhat from data
      6.7_kg, // about 14.8 lbs
      // 1.36f); // motor count - determined somewhat from data
      // 2.5f); // motor count - determined somewhat from data
      3.0f); // motor count - determined somewhat from data

    LinearConstraints linear_constraints(
      70_inps, // max vel - for testing
      // 20.0_inps2, // max accel - for testing
      10000.0_inps2, // max accel - for testing
      // 150_inps2 // max decel - for testing also
      200_inps2 // max decel - for testing also
    );
    //
    // // TODO: what is the difference between angular accel/decel?
    // AngularConstraints
    // angular_constraints(2.0_radps, 1.3_radps2, 1.3_radps2);
    AngularConstraints angular_constraints(2.0_radps,
                                           // 1.3_radps2,
                                           // 1.3_radps2

                                           2.0_radps2,
                                           2.0_radps2);
    //
    Constraints constraints(robot_constraints,
                            linear_constraints,
                            angular_constraints);
    //
    // bool debug = true;
    bool debug = false;
    //
    std::shared_ptr<Trajectory> test_trajectory(
      new Trajectory(spline_ptr,
                     constraints,
                     {},
                     {},
                     // some initial velocity for it to move?
                     // TODO: could there be a place on the curve that also has
                     // a velof zero? if so this would also have the same issue?
                     0_inps,
                     0_inps,
                     0.1_in,
                     debug));

    // trajectoryDebugPrint(test_trajectory.get());

    // print out final trajectory and debug info

    // drivetrain.setBrakeMode(pros::v5::MotorBrake::hold);

    // std::cout << "running path!" << std::endl;
    // use path follow to follow the path
    lyfast::PathFollow(controllers, vexmaps_chassis, test_trajectory)
        // .drive_toleranceDuration(100_sec)
        // .drive_largeToleranceDuration(100_sec)
        .lookahead(20_msec + drivetrain_config.input_delay)
        .reverse()
        // .parameterization(blazing::lyfast::time_based)
        .timeout(5_sec) |
      run;
}

void run_auton() {
    // runs before anything else
    pre_auton();

    units::V2Position centerTopGoalFirst = { -9.5_in, 8.0_in };

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    // start auton
    RobotSetPose(-46.57, -14, 90);

    std::cout << "Stated auto: " << std::endl;

    intake::in();

    bool pushing = true;

    if (pushing) mb.moveTo(-46.57, -4.7).timeout(1.2_sec) | chain;

    mb.moveTo(-46.376, -46.1)
        .only_y(true)
        .reverse()
        // .drive_errorTolerance(2.0_in)
        // .customAngularLinearFunc([](Angle angle) {
        //     return units::cos(angle);
        // })
        .timeout(1.3_sec) |
      chain;
    chain.wait();

    matchload(-1, -1, 0.5_sec);
    score_long_goal(-1, -1, 1_sec);

    matchloader::up();

    // mb.turnTo(-1_tile, -1_tile).radius(-1.2) | chain;

    // small swing
    drivetrain.moveTank(1.0_volt, -0.8_volt);
    pros::delay(200);

    // turn towards balls
    mb.turnTo(-1_tile, -1_tile)
        .constantVelocity(-10_inps)
        .turn_errorTolerance(10.0_stDeg)
        .turn_velocityTolerance(400_radps)
        .turn_toleranceDuration(0_sec)
        .turn_chainErrorTolerance(10_stDeg) |
      chain;

    // move towards lower left balls
    mb.moveTo(-1_tile, -1_tile)
        // assume already going with some momentum
        .drive_vel_accelSlew(300_inps)
        .executeAfterMotion([] {
            intake::in();
        })
        .drive_vel_minVel(30_inps) |
      chain;

    mb.moveTo(-1_tile, 1_tile)
        .drive_vel_mp_maxVel(57_inps)
        // .drive_vel_mp_setMaxAccel(70_inps2)
        // already going fast from previous motion, slew can be faster
        .drive_vel_accelSlew(300_inps) |
      chain;

    // mb.turnTo(centerTopGoalFirst.x, centerTopGoalFirst.y).reverse() | chain;

    // mb.moveTo(centerTopGoalFirst.x, centerTopGoalFirst.y)
    //     .reverse()
    //     .closeThreshold(4_in)
    //   // .drive_vel_maxVel(40_inps)
    //   | chain;
    // mb.turnTo(11.574, -11.401).reverse() | chain;
    mb.moveTo(-11.7, 11.0).reverse() | chain;

    // turn towards correct heading
    // mb.turnTo(135)
    //     .constantVelocity(-5_inps)
    //
    //     // infinite time motion
    //     .turn_toleranceDuration(100_sec)
    //     .turn_largeToleranceDuration(100_sec)
    //     .timeout(1_sec) |
    //   chain;

    chain.waitUntil(closeEnough({ -1_tile, -1_tile }, 7_in));
    intake::in();
    chain.waitUntil(closeEnough({ -1_tile, 1_tile }, 14_in));
    matchloader::down();
    chain.waitUntil(closeEnough({ -11.6_in, 11.0_in }, 4_in));
    // start scoring
    // wait for balls to come up the intake
    pros::delay(200);
    // score
    intake::score_middle();
    // score for some time
    pros::delay(1000);

    // stop scoring, intake again
    intake::in();

    // exit any remaining motions
    chain.exitAll();

    // go towards matchloader
    mb.moveTo(-48_in, 46).drive_errorTolerance(2.0_in).only_y(true) | run;

    // turn to and go to matchloader

    matchload(-1, 1, 0.5_sec);
    silly();

    // score_long_goal(-1, 1, 1_sec);
}

} // namespace awp
