/**
 * @file
 * @brief tuning utils
 */

#include "tuning.h"
#include "autos.h"
#include "globals/blazing_globals.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
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

    int pct = 0.5;

    left_motors.move_voltage(12000 * pct);
    right_motors.move_voltage(12000 * pct);

    while (true) {
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
                                 time_difference.convert(sec))
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
                std::cout << std::format("decreased kp to {}", curr_kp)
                          << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_L1)) {
                curr_kp += kp_delta;
                std::cout << std::format("increased kp to {}", curr_kp)
                          << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R2)) {
                curr_kd -= kd_delta;
                std::cout << std::format("decreased kd to {}", curr_kd)
                          << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R1)) {
                curr_kd += kd_delta;
                std::cout << std::format("increased kd to {}", curr_kd)
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

    double kp_delta = 0.05;
    double kd_delta = 0.05;

    while (true) {
        RobotSetPose(0, 0, 0);

        auto start_time = from_msec(pros::millis());

        mb.moveTo(target_distance, 0_in).drive_kp(curr_kp).drive_kd(curr_kd) |
          run;

        auto end_time = from_msec(pros::millis());

        auto time_difference = end_time - start_time;

        auto total_error = units::V2Position(target_distance, 0_in)
                             .distanceTo(tracker.getPosition());
        auto forwards_error = target_distance - tracker.getPosition().x;
        auto sideways_error = target_distance - tracker.getPosition().y;

        std::cout
          << std::format(
               "final error: {:.3f}, forwards: {:.3f}, sideways: {:.3f}",
               total_error,
               forwards_error,
               sideways_error)
          << std::endl;

        std::cout << std::format("position: {:.3f} {:.3f} {:.3f}",
                                 tracker.getPosition().x.convert(in),
                                 tracker.getPosition().y.convert(in),
                                 tracker.getAngle().convert(deg))
                  << std::endl;

        std::cout << std::format("took {:.4f} time to finish turn",
                                 time_difference.convert(sec))
                  << std::endl;

        while (
          !controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_A)) {
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_LEFT)) {
                target_distance -= target_distance_delta;
                std::cout << std::format("decreased target to {}",
                                         target_distance)
                          << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_RIGHT)) {
                target_distance += target_distance_delta;
                std::cout << std::format("increased target to {}",
                                         target_distance)
                          << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_L2)) {
                curr_kp -= kp_delta;
                std::cout << std::format("decreased kp to {}", curr_kp)
                          << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_L1)) {
                curr_kp += kp_delta;
                std::cout << std::format("increased kp to {}", curr_kp)
                          << std::endl;
            }

            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R2)) {
                curr_kd -= kd_delta;
                std::cout << std::format("decreased kd to {}", curr_kd)
                          << std::endl;
            }
            if (controller.get_digital_new_release(
                  pros::E_CONTROLLER_DIGITAL_R1)) {
                curr_kd += kd_delta;
                std::cout << std::format("increased kd to {}", curr_kd)
                          << std::endl;
            }
            pros::delay(10);
        }
    }
}
