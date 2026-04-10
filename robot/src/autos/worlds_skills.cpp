/**
 * @file
 * @brief :)))) */

#include "apis.h"
//
#include "autos.h"
#include "blazing/utils.hpp"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/config.h"
#include "globals/device_globals.h"
#include "lyfast/motion_profiling/mp.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include <iostream>
#include <tuple>

namespace skills {

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

    mb.moveTo(35_in * sign_x, long_goal * sign_y).reverse() | run;

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

std::shared_ptr<lyfast::geometry::Spline> makeSpline(
  const std::vector<std::shared_ptr<lyfast::geometry::Curve>>& curves) {
    return std::shared_ptr<lyfast::geometry::Spline> {
        new lyfast::geometry::Spline(curves)
    };
}

namespace skills_paths {
auto start_TO_in_red_park = line(-44.125, 0.039, -61.257, 0.363);
auto in_red_park_TO_out_of_red = line(-61.257, 0.363, -44.365, 0);
auto out_of_red_TO_get_blue_middle = line(-44.365, 0, -14.475, 8.582);
auto get_blue_middle_TO_End_Control = line(-14.475, 8.582, -15.992, 15.457);
auto End_Control_TO_score_middle = line(-15.992, 15.457, -13.179, 12.513);
auto score_middle_TO_ull = line(-13.179, 12.513, -41.032, 46.515);
auto ull_TO_uls = line(-41.032, 46.515, -30.06, 46.931);
auto uls_TO_ulm = line(-30.06, 46.931, -57.173, 46.6);
auto ulm_TO_url1 =
  curve(-57.173, 46.6, -37.078, 46.6, -49.519, 65.882, 22.979, 59.5);
auto url1_TO_urls = line(22.979, 59.5, 38.085, 46.543);
auto urls_TO_urm = line(38.085, 46.543, 56.959, 46.195);
auto urm_TO_urls2 = line(56.959, 46.195, 30.3, 46.442);
auto urls2_TO_End_Control = line(30.3, 46.442, 36.735, 45.597);
auto End_Control_TO_ur_cluster =
  curve(36.735, 45.597, 38.483, 43.58, 35.403, 36.383, 30.449, 30.352);
auto ur_cluster_TO_End_Control = line(30.449, 30.352, 34.897, 33.991);
auto End_Control_TO_blue_park = line(34.897, 33.991, 44.859, -0.234);
auto blue_park_TO_in_blue_park = line(44.859, -0.234, 61.945, -0.234);
auto in_blue_park_TO_blue_park2 = line(61.945, -0.234, 45.304, -0.056);
auto blue_park2_TO_go_bottom = line(45.304, -0.056, 16.95, 18.346);
auto go_bottom_TO_bottom_score = line(16.95, 18.346, 11.967, 12.295);
auto bottom_score_TO_back_bottom = line(11.967, 12.295, 16.594, 16.389);
auto back_bottom_TO_dr_cluster = line(16.594, 16.389, 23.741, -23.427);
auto dr_cluster_TO_drl = line(23.741, -23.427, 39.523, -46.844);
auto drl_TO_drls = line(39.523, -46.844, 29.949, -47.059);
auto drls_TO_drm = line(29.949, -47.059, 57.173, -46.6);
auto drm_TO_dll =
  curve(57.173, -46.6, 37.078, -46.6, 49.519, -65.882, -22.979, -59.5);
auto dll_TO_dls = line(-22.979, -59.5, -35.563, -47.505);
auto dls_TO_dlm = line(-35.563, -47.505, -56.502, -47.295);
auto dlm_TO_dls2 = line(-56.502, -47.295, -30.812, -46.393);
auto dls2_TO_ending =
  curve(-30.812, -46.393, -65.874, -35.503, -61.239, -20.894, -62.307, -0.693);
} // namespace skills_paths

auto long_match_curve_top = skills_paths::ulm_TO_url1;
auto long_match_curve_bottom = skills_paths::drm_TO_dll;
auto park_curve = skills_paths::dls2_TO_ending;

std::shared_ptr<lyfast::mp::Trajectory>
makeTrajectory(std::shared_ptr<lyfast::geometry::Curve> curve) {
    using namespace blazing::lyfast;
    using namespace blazing::lyfast::geometry;
    using namespace blazing::lyfast::mp;

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
    std::shared_ptr<Trajectory> trajectory(
      new Trajectory(curve,
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
    return trajectory;
}

auto pathFollow(std::shared_ptr<lyfast::mp::Trajectory> trajectory) {
    auto motion = lyfast::PathFollow(controllers, vexmaps_chassis, trajectory);
    std::ignore = motion.lookahead(20_msec + drivetrain_config.input_delay)
                    .reverse()
                    .timeout(5_sec);
    return motion;
}

auto pathFollow(std::shared_ptr<lyfast::geometry::Curve> curve) {
    return pathFollow(makeTrajectory(curve));
}

void run_auton() {
    // runs before anything else
    pre_auton();

    units::V2Position centerTopGoalFirst = { -9.5_in, 8.0_in };

    Length long_goal = 47.1_in;
    Length normal_match = 46.7_in;

    // start auton
    RobotSetPose(-44.365, 0, 180);

    intake::in();

    mb.moveTo(-15.992, 15.457).reverse() | run;
    // turn to and move to middle goal
    mb.turnTo(-13.179, 12.513).reverse() | run;
    mb.moveTo(-13.179, 12.513).reverse() | run;

    // move towards long goal, forwards
    mb.moveTo(-41.032, long_goal) | run;

    // turn to and move there
    mb.turnTo(-32.032, long_goal).reverse() | run;
    mb.moveTo(-32.032, long_goal).reverse() | run;

    // go to matchload
    matchload(-1, 1, 2.0_sec);

    // follow path to go to other side
    pathFollow(long_match_curve_top) | run;

    // move towards long goal
    mb.moveTo(41.032, long_goal).reverse() | run;

    // turn to and move there
    mb.turnTo(32.032, long_goal).reverse() | run;
    mb.moveTo(32.032, long_goal).reverse() | run;

    matchload(1, 1, 2_sec);
    score_long_goal(1, 1, 2_sec);

    // move forwards a tiny amount
    drivetrain.moveTank(1_volt, 1_volt);
    pros::delay(120);

    // get one red ball from cluster
    mb.turnTo(30.449, 30.352) | run;
    mb.moveTo(30.449, 30.352) | run;

    // go back tiny amount
    drivetrain.moveTank(-1_volt, -1_volt);
    pros::delay(100);

    // move towards park
    mb.turnTo(43, 0) | run;
    mb.moveTo(43, 0) | run;

    // TODO: get balls from park

    // move from park to score on bottom goal
    // blows up cluster
    mb.moveTo(22.754, 22.033).reverse() | run;

    // turn to and score
    mb.turnTo(11.967, 12.295) | run;
    mb.moveTo(11.967, 12.295) | run;

    // score
    pros::delay(3000);

    // go back tiny amount
    drivetrain.moveTank(-1_volt, -1_volt);
    pros::delay(130);

    // move towards long goal, forwards
    mb.moveTo(41.032, -long_goal) | run;

    // turn to and move there
    mb.turnTo(32.032, -long_goal).reverse() | run;
    mb.moveTo(32.032, -long_goal).reverse() | run;

    // go to matchload
    matchload(1, -1, 2.0_sec);

    // follow path to go to other side
    pathFollow(long_match_curve_bottom) | run;

    // move towards long goal
    mb.moveTo(-41.032, -long_goal).reverse() | run;

    // turn to and move there
    mb.turnTo(-32.032, -long_goal).reverse() | run;
    mb.moveTo(-32.032, -long_goal).reverse() | run;

    matchload(-1, -1, 2_sec);
    score_long_goal(-1, -1, 2_sec);

    // go park
    pathFollow(park_curve) | run;
}

} // namespace skills
