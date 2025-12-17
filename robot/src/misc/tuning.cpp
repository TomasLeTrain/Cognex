/**
 * @file
 * @brief tuning utils
 */

#include "tuning.h"
#include "autos.h"
#include "globals/blazing_globals.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "screen/screen.h"
#include "units/Vector2D.hpp"
#include <cmath>

void odom_tuning() {
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

        Length target_distance_in = 2_tile;

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
    // units::Pose pose = { 0_in, 0_in };
    // model_manager.setPose(pose);
    // tracker.setPose(pose);

    sideways_odom_rotation.set_position(0);
    forwards_odom_rotation.set_position(0);
    pros::delay(10);

    float pct = 0.5;

    while (true) {
        left_motors.move_voltage(12000 * pct);
        right_motors.move_voltage(-12000 * pct);

        units::V2Position deltas = { forwards_tracker.getDelta(),
                                     sideways_tracker.getDelta() };
        Angle delta_theta = imu_tracker.getDelta();

        units::V2Position offsets = deltas / to_stRad(delta_theta);

        std::cout << offsets.x.convert(in) << " " << offsets.y.convert(in)
                  << std::endl;

        pros::delay(10);
    }
}

void turn_pid_tuning() {
    double target_theta = 90;
    // by how much we can increase or decrease
    double target_theta_delta = 45;

    double curr_kp = angular_pid.get_kp() / angular_pid.UKP;
    double curr_kd = angular_pid.get_kd() / angular_pid.UKD;

    double kp_delta = 0.05;
    double kd_delta = 0.05;

    while (true) {
        RobotSetPose(0, 0, 0);
        auto start_time = from_msec(pros::millis());

        mb.turnTo(target_theta).turn_kp(curr_kp).turn_kd(curr_kd) | run;

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
            pros::delay(10);
        }
    }
}

void drive_pid_tuning() {
    Length target_distance = 48_in;
    Length target_distance_delta = 12_in;

    double curr_kp = linear_pid.get_kp() / linear_pid.UKP;
    double curr_kd = linear_pid.get_kd() / linear_pid.UKD;

    Voltage curr_accel_slew = 1_volt;
    Number curr_k_lat = 0.0;

    double kp_delta = 0.05;
    double kd_delta = 0.05;
    Voltage slew_delta = 0.025_volt;
    Number k_lat_delta = 0.01;

    bool k_lat_config_active = false;

    bool reversed = false;

    units::Pose start_pose = { -24_in, -24_in, 0_stDeg };

    RobotSetPose(start_pose);

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    while (true) {

        auto start_time = from_msec(pros::millis());

        if (reversed) {
            // RobotSetPose(2 * target_distance.convert(in), 0, 0);
            mb.moveTo(start_pose.x + target_distance, start_pose.y)
                .drive_kp(curr_kp)
                .drive_kd(curr_kd)
                .drive_accelSlew(curr_accel_slew)
                .k_lat(curr_k_lat)
                .reverse() |
              run;
        } else {
            // RobotSetPose(0, 0, 0);
            mb.moveTo(start_pose.x + target_distance, start_pose.y)
                .drive_kp(curr_kp)
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
                    curr_accel_slew += slew_delta;
                    std::cout << std::format("increased slew to {:.3f}",
                                             curr_accel_slew.internal())
                              << std::endl;
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
                    curr_accel_slew -= slew_delta;
                    std::cout << std::format("decreased slew to {:.3f}",
                                             curr_accel_slew.internal())
                              << std::endl;
                }
            }
            // kp = 7
            // kd = 10.5
            pros::delay(10);
        }
    }
}
