/**
 * @file
 * @brief tuning utils
 */

#include "tuning.h"
#include "autos.h"
#include "blazing/utils.hpp"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/config.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "screen/screen.h"
#include "systems/matchloader.h"
#include "units/Vector2D.hpp"
#include <cmath>

// blocking
void odom_diameter_tuning() {
    // units::Pose pose = { 0_in, 0_in };
    // model_manager.setPose(pose);
    // tracker.setPose(pose);

    sideways_odom_rotation.set_position(0);
    forwards_odom_rotation.set_position(0);
    pros::delay(10);

    while (true) {
        // auto blazing_position = tracker.getPosition();
        // auto blazing_theta = tracker.getAngle();
        //
        // auto vexmaps_pose = model_manager.getPose();

        Length target_distance_in = 48_in;

        auto odom_to_diameter = [target_distance_in](double pos) -> Length {
            // const Length diameter = 2.0_in;
            // const Length circumerence = diameter * M_PI;
            const double rotations = (pos * deg / 100.0) / rot;

            // const Length measured = rotations * circumerence;

            const Length expected_diameter =
              target_distance_in / (M_PI * rotations);
            return expected_diameter;
        };

        screen::health::set_console_text(std::format(
          "if x or y are negative then offets should be flipped.\n"
          "distance: {}_in\n"
          "sideways: {:.4f}\n"
          "forward: {:.4f}\n",
          target_distance_in.convert(in),
          odom_to_diameter(sideways_odom_rotation.get_position()).convert(in),
          odom_to_diameter(forwards_odom_rotation.get_position()).convert(in)));

        // maybe unneeded?
        pros::delay(50);
    }
}

void odom_offset_tuning() {
    sideways_odom_rotation.set_position(0);
    forwards_odom_rotation.set_position(0);
    pros::delay(10);

    float pct = 0.5;

    double last_sideways_rotation = sideways_odom_rotation.get_position();
    double last_forwards_rotation = forwards_odom_rotation.get_position();
    Angle last_angle = RobotGetPose().orientation;

    auto get_offset =
      [](double distance_delta, Length wheel_diameter, Angle angle_delta) {
          const double rotations = (distance_delta * deg / 100.0) / rot;

          const Length measured = rotations * (wheel_diameter * M_PI);
          return measured / to_stRad(angle_delta);
      };

    Time last_measurement_time = now();

    while (true) {
        left_motors.move_voltage(-12000 * pct);
        right_motors.move_voltage(12000 * pct);

        // units::V2Position deltas = { forwards_tracker.getDelta(),
        //                              sideways_tracker.getDelta() };
        // Angle delta_theta = imu_tracker.getDelta();
        //
        // units::V2Position offsets = deltas / to_stRad(delta_theta);

        // gets offsets every 0.2 seconds
        if (blazing::timeoutDone(0.2_sec, last_measurement_time)) {
            last_measurement_time = now();
            Angle angle_delta = RobotGetPose().orientation - last_angle;
            last_angle = RobotGetPose().orientation;

            double curr_forwards_rotation =
              forwards_odom_rotation.get_position();
            double curr_sideways_rotation =
              sideways_odom_rotation.get_position();

            double forwards_delta =
              curr_forwards_rotation - last_forwards_rotation;
            double sideways_delta =
              curr_sideways_rotation - last_sideways_rotation;

            last_forwards_rotation = curr_forwards_rotation;
            last_sideways_rotation = curr_sideways_rotation;

            units::V2Position offsets = {
                get_offset(forwards_delta,
                           forwards_tracker_config.diameter,
                           angle_delta),
                get_offset(sideways_delta,
                           sideways_tracker_config.diameter,
                           angle_delta)
            };

            std::cout << offsets.x.convert(in) << " " << offsets.y.convert(in)
                      << std::endl;
        }

        pros::delay(20);
    }
}

void turn_pid_tuning() {
    double target_theta = 90;
    // by how much we can increase or decrease
    double target_theta_delta = 45;

    double curr_kp = turn_drive_pid.get_kp() / turn_drive_pid.UKP;
    double curr_ki = turn_drive_pid.get_ki() / turn_drive_pid.UKI;
    double curr_kd = turn_drive_pid.get_kd() / turn_drive_pid.UKD;

    double kp_delta = 0.05;
    double ki_delta = 0.01;
    double kd_delta = 0.05;

    while (true) {
        RobotSetPose(0, 0, 0);
        auto start_time = from_msec(pros::millis());

        mb.turnTo(target_theta)
            .turn_kp(curr_kp)
            .turn_ki(curr_ki)
            .turn_kd(curr_kd) |
          run;

        auto end_time = from_msec(pros::millis());

        auto time_difference = end_time - start_time;

        std::cout << std::format("final error was {:.3f}",
                                 target_theta - tracker.getAngle().convert(deg))
                  << std::endl;

        std::cout << std::format("position: {:.2f} {:.2f} {:.4f}",
                                 tracker.getPosition().x.convert(in),
                                 tracker.getPosition().y.convert(in),
                                 tracker.getAngle().convert(deg))
                  << std::endl;

        std::cout << std::format("took {:.4f} time to finish turn",
                                 time_difference.convert(msec))
                  << std::endl;

        while (
          !controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_A)) {
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_LEFT)) {
                target_theta -= target_theta_delta;
                std::cout << std::format("decreased to {}", target_theta)
                          << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                target_theta += target_theta_delta;
                std::cout << std::format("increased to {}", target_theta)
                          << std::endl;
            }

            if (controller.get_digital_new_release(controls::DOWN)) {
                curr_ki -= ki_delta;
                std::cout << std::format("ki - to {}", curr_ki) << std::endl;
            }
            if (controller.get_digital_new_release(controls::UP)) {
                curr_ki += ki_delta;
                std::cout << std::format("ki + to {}", curr_ki) << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_L2)) {
                curr_kp -= kp_delta;
                std::cout << std::format("decreased kp to {:.3f}", curr_kp)
                          << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_L1)) {
                curr_kp += kp_delta;
                std::cout << std::format("increased kp to {:.3f}", curr_kp)
                          << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R2)) {
                curr_kd -= kd_delta;
                std::cout << std::format("decreased kd to {:.3f}", curr_kd)
                          << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R1)) {
                curr_kd += kd_delta;
                std::cout << std::format("increased kd to {:.3f}", curr_kd)
                          << std::endl;
            }
            if (controller.get_digital_new_release(controls::X)) {
                matchloader::set(!matchloader::get());
            }

            pros::delay(10);
        }
    }
}

void drive_pid_tuning() {
    Length target_distance = 24_in;
    Length target_distance_delta = 12_in;

    double curr_kp = linear_pid.get_kp() / linear_pid.UKP;
    double curr_ki = linear_pid.get_ki() / linear_pid.UKI;
    double curr_kd = linear_pid.get_kd() / linear_pid.UKD;

    Voltage curr_accel_slew = 1_volt;
    Number curr_k_lat = 0.0;

    double kp_delta = 0.05;
    double kd_delta = 0.05;
    double ki_delta = 0.01;

    Voltage slew_delta = 0.025_volt;
    Number k_lat_delta = 0.01;

    bool k_lat_config_active = false;

    bool reversed = false;

    // units::Pose start_pose = { -24_in, -24_in, 0_stDeg };
    units::Pose start_pose = { 0_in, 0_in, 0_stDeg };

    RobotSetPose(start_pose);

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    while (true) {
        RobotSetPose(0, 0, 0);

        std::cout << std::format("start is {:.3f} {:.3f}",
                                 RobotGetPose().x.convert(in),
                                 RobotGetPose().y.convert(in))
                  << std::endl;

        auto start_time = from_msec(pros::millis());

        if (reversed) {
            // RobotSetPose(2 * target_distance.convert(in), 0, 0);
            mb.moveTo(start_pose.x + target_distance, start_pose.y)
                .drive_kp(curr_kp)
                .drive_ki(curr_ki)
                .drive_kd(curr_kd)
                .drive_accelSlew(curr_accel_slew)
                .k_lat(curr_k_lat)
                .reverse() |
              run;
        } else {
            // RobotSetPose(0, 0, 0);
            mb.moveTo(start_pose.x + target_distance, start_pose.y)
                .drive_kp(curr_kp)
                .drive_ki(curr_ki)
                .drive_kd(curr_kd)
                .k_lat(curr_k_lat)
                .drive_accelSlew(curr_accel_slew) |
              run;
        }

        auto end_time = from_msec(pros::millis());

        auto time_difference = end_time - start_time;

        auto curr_pose = RobotGetPose();
        auto error_vec =
          units::V2Position(start_pose.x + target_distance, start_pose.y) -
          curr_pose;

        auto total_error = error_vec.magnitude();
        auto forwards_error = error_vec.x;
        auto sideways_error = error_vec.y;

        std::cout
          << std::format(
               "final error: {:.3f}, forwards: {:.3f}, sideways: {:.3f}",
               total_error.convert(in),
               forwards_error.convert(in),
               sideways_error.convert(in))
          << std::endl;

        std::cout << std::format("position: {:.3f} {:.3f} {:.3f}",
                                 curr_pose.x.convert(in),
                                 curr_pose.y.convert(in),
                                 curr_pose.orientation.convert(deg))
                  << std::endl;

        std::cout << std::format("took {:.4f} time to finish turn",
                                 time_difference.convert(sec))
                  << std::endl;

        while (
          !controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_A)) {
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_LEFT)) {
                target_distance -= target_distance_delta;
                std::cout << std::format("decreased target to {:.3f}",
                                         target_distance.convert(in))
                          << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_B)) {
                if (reversed) {
                    mb.moveTo(start_pose.x, start_pose.y)
                        .drive_maxVolt(0.6_volt) |
                      async;
                } else {
                    mb.moveTo(start_pose.x, start_pose.y)
                        .drive_maxVolt(0.6_volt)
                        .reverse() |
                      async;
                }
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_X)) {

                reversed = !reversed;

                // turns around
                if (reversed) {
                    mb.turnTo(0) | async;
                } else {
                    mb.turnTo(180) | async;
                }
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_Y)) {
                k_lat_config_active = !k_lat_config_active;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                target_distance += target_distance_delta;
                std::cout << std::format("increased target to {:.3f}",
                                         target_distance.convert(in))
                          << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_L2)) {
                curr_kp -= kp_delta;
                std::cout << std::format("decreased kp to {:.3f}", curr_kp)
                          << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_L1)) {
                curr_kp += kp_delta;
                std::cout << std::format("increased kp to {:.3f}", curr_kp)
                          << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R2)) {
                curr_kd -= kd_delta;
                std::cout << std::format("decreased kd to {:.3f}", curr_kd)
                          << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R1)) {
                curr_kd += kd_delta;
                std::cout << std::format("increased kd to {:.3f}", curr_kd)
                          << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_UP)) {
                if (k_lat_config_active) {
                    curr_k_lat += k_lat_delta;
                    std::cout << std::format("increased klat to {:.3f}",
                                             curr_k_lat.internal())
                              << std::endl;
                } else {
                    curr_ki += ki_delta;
                    std::cout << std::format("increased ki to {:.3f}", curr_ki)
                              << std::endl;
                    // curr_accel_slew += slew_delta;
                    // std::cout << std::format("increased slew to {:.3f}",
                    //                          curr_accel_slew.internal())
                    //           << std::endl;
                }
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_DOWN)) {
                if (k_lat_config_active) {
                    curr_k_lat -= k_lat_delta;
                    std::cout << std::format("decreased klat to {:.3f}",
                                             curr_k_lat.internal())
                              << std::endl;
                } else {
                    curr_ki -= ki_delta;
                    std::cout << std::format("decreased ki to {:.3f}", curr_ki)
                              << std::endl;
                    // curr_accel_slew -= slew_delta;
                    // std::cout << std::format("decreased slew to {:.3f}",
                    //                          curr_accel_slew.internal())
                    //           << std::endl;
                }
            }
            // kp = 7
            // kd = 10.5

            // kp = 6.3
            // kd = 10.2
            pros::delay(10);
        }
    }
}
