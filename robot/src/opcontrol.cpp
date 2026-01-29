#include "apis.h"
//

#include "autos.h"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
#include "main.h"
#include "screen/screen.h"
#include "systems/drivetrain.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/piston.h"
#include "systems/sysid.h"
#include "systems/wings.h"
#include "tuning.h"

void manual_mp_test() {
    std::vector<std::pair<LeftRightSpeeds, LeftRightSpeeds>> data;
    std::vector<LeftRightVoltages> voltages;

    Time start_time = from_msec(pros::millis());

    LinearAcceleration max_acceleration = 150_inps2;
    LinearVelocity max_velocity = 60_inps;
    Length distance = 48_in;

    // time to reach max velocity
    Time accel_time = (max_velocity / max_acceleration);

    Length accel_dist = 0.5 * max_acceleration * accel_time * accel_time;
    Length steady_dist = distance - 2 * accel_dist;

    Time steady_time = steady_dist / max_velocity;

    // decel_start_time = total time - time to decelerate to zero
    Time decel_start_time = accel_time + steady_time;

    Time end_time = steady_time + 2 * accel_time;

    while (true) {
        Time curr_time = from_msec(pros::millis());
        Time motion_time = curr_time - start_time;

        if (motion_time >= end_time) {
            break;
        }

        // double throttle =
        // master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y); double turn
        // = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // //
        // throttle /= 127.0;
        // turn /= 127.0;
        //
        // // constexpr auto max_vel = (7.5_rpm * 3.25_in * M_PI / rad);
        // constexpr auto max_vel = 2_mps;
        // // constexpr auto max_rad = 2_mps * ;
        // //
        // DifferentialSpeeds target {
        //     .linear_velocity = throttle * max_vel,
        //     .angular_velocity = turn * rad * (max_vel / (track_width *
        //     0.5))
        // };

        auto mp = [&](Time time) -> LinearVelocity {
            // assume velocity of 0 everywhere outside mp
            if (time > end_time || time < 0_sec) return 0_mps;

            if (steady_dist < 0.0_in) {
                if (time < end_time / 2) {
                    return (max_acceleration * time);
                } else {
                    return (-max_acceleration * (time - end_time * 0.5) +
                            // max vel we got to
                            max_acceleration * (end_time * 0.5));
                }
            } else {
                if (time < accel_time) {
                    return (max_acceleration * time);
                } else {
                    if (time <= decel_start_time) {
                        return (max_velocity);
                    } else {
                        return (-max_acceleration * (time - decel_start_time) +
                                max_velocity);
                    }
                }
            }
            // targets velocity 10 msec into the future
        };

        // target speed in the future
        LinearVelocity curr_target_speed = mp(motion_time + 10_msec);

        // use desired speed now in logs
        LinearVelocity desired_curr_speed = mp(motion_time);

        DifferentialSpeeds target { .linear_velocity = curr_target_speed,
                                    .angular_velocity = 0_radps };

        DifferentialSpeeds desired_target { .linear_velocity =
                                              desired_curr_speed,
                                            .angular_velocity = 0_radps };

        LinearVelocity curr_left_vel =
          drivetrain.getDrivetrainVelocities().left_vel;
        LinearVelocity curr_right_vel =
          drivetrain.getDrivetrainVelocities().right_vel;

        LinearVelocity curr_lin_vel = (curr_left_vel + curr_right_vel) / 2.0;

        LeftRightVoltages volts =
          controllers.velocity_feedforward.update(target, 10_msec);
        // controllers.velocity_feedforward.update(
        //   { curr_left_vel, curr_right_vel },
        //   target,
        //   10_msec);

        drivetrain.moveTank(volts.left_voltage, volts.right_voltage);

        data.emplace_back(
          LeftRightSpeeds { desired_curr_speed, desired_curr_speed },
          LeftRightSpeeds { curr_left_vel, curr_right_vel });
        voltages.emplace_back(volts);

        pros::delay(10);
    }

    drivetrain.setBrakeMode(pros::MotorBrake::hold);

    while (true) {
        left_motors.move(0);
        right_motors.move(0);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            // std::cout << "vel data: " << std::endl;
            // lyfast::printData(data);
            std::cout << "LEFT MOTORS: " << std::endl;
            std::cout << "\\left[";
            for (auto [target, actual] : data) {
                std::cout << "\\left(" << target.left_vel.internal() << ","
                          << actual.left_vel.internal() << "\\right),";
            }
            std::cout << "\\right]" << std::endl;

            std::cout << "RIGHT MOTORS: " << std::endl;
            std::cout << "\\left[";
            for (auto [target, actual] : data) {
                std::cout << "\\left(" << target.right_vel.internal() << ","
                          << actual.right_vel.internal() << "\\right)";
            }
            std::cout << "\\right]," << std::endl;

            std::cout << "VOLTAGES:" << std::endl;
            std::cout << "\\left[";
            for (auto curr_voltages : voltages) {
                std::cout << "\\left(" << curr_voltages.left_voltage.internal()
                          << "," << curr_voltages.right_voltage.internal()
                          << "\\right),";
            }

            std::cout << "\\right]" << std::endl;
        }
        pros::delay(10);
    }
}

void opcontrol() {

    // goated linear pid:
    // kp: 6.9
    // ki: 0.3
    // kd: 9.9

    //    RobotSetPose(-48, -48, 180);
    //
    // intake::init(false);
    // long_goal_tuning();
    // return;

    // turn_pid_tuning();
    // drive_pid_tuning();
    // return;
    // autonomous();
    // return;

    // odom_offset_tuning();

    linear_ka_kp_ki_tuner();
    manual_mp_test();
    return;

    // matchloader::init(false);
    // turn_pid_tuning();
    // return;

    // RobotSetPose(10, 10, 0);

    // cancel any auton motions that could be currently running

    // linear_ka_kp_ki_tuner();
    // create_accel_data({ 0.5_volt, 0.5_volt, 2_sec }, "Linear");

    intake::init(true);
    matchloader::init(true);
    wings::init(true);
    odom_retract::init(true);

    intake::setDriverColorSort(false);

    // no need to initialize in auto
    drivetrain.setBrakeMode(pros::MotorBrake::coast);

    odom_retract::retractOdom();

    // RobotSetPose(-48, -48, 180);

    // matchloadTuning();
    // long_goal_tuning();
    //

    // runs exclusively inside opcontrol to guarantee it does not interfer with
    // autos (stopped automatically when not in driver mode)
    while (true) {
        base::driveUpdate();
        pros::delay(10);
    }
}
