#include "apis.h"
//

#include "globals.h"
#include "globals/device_globals.h"
#include "pros/device.hpp"
#include "screen/screen.h"

namespace health_daemon {
// avoids spamming notifications
// every error only every produces one notification
std::vector<bool> port_dc(22, false);
bool imu_invalid = false;

void health_task() {
    // here we constantly check for any misconfigurations in devices and
    // subsystems

    auto dc_not_processed = [&](auto& device) {
        return !port_dc[device.get_port()];
    };
    auto update_dc = [&](auto& device) {
        port_dc[device.get_port()] = true;
    };

    auto check_motor_group = [](pros::MotorGroup& motors,
                                std::string motor_group_name) {
        for (auto port : motors.get_port_all()) {
            auto zero_indexed_port = port - 1;
            bool installed =
              pros::DeviceType::motor ==
              (pros::DeviceType)pros::c::registry_get_plugged_type(
                zero_indexed_port);
            if (!installed && !port_dc[port]) {
                screen::health::add_notification(
                  std::format("Port {}: Motor Unplugged!", port),
                  std::format(
                    "Motor on {} motor group. Make sure this is not crucial!",
                    motor_group_name),
                  screen::health::warn);
            }
        }
    };

    check_motor_group(left_motors, "left");
    check_motor_group(right_motors, "right");

    // check imu dc / invalid heading
    if (!imu.is_installed() && dc_not_processed(imu)) {
        update_dc(imu);

        screen::health::add_notification(
          std::format("Port {}: IMU Unplugged!", imu.get_port()),
          "This is likely very bad!",
          screen::health::critical);
    }

    if (!imu.is_calibrating() && !std::isfinite(imu.get_rotation()) &&
        !imu_invalid) {
        imu_invalid = true;
        screen::health::add_notification(
          std::format("Port {}: IMU returns INF!", imu.get_port()),
          "This is likely very bad!",
          screen::health::critical);
    }

    // check rotation sensors dc
    if (!forwards_odom_rotation.is_installed() &&
        dc_not_processed(forwards_odom_rotation)) {
        update_dc(forwards_odom_rotation);

        screen::health::add_notification(
          std::format("Port {}: forwards rotation Unplugged!", imu.get_port()),
          "This is likely very bad!",
          screen::health::critical);
    }
}

void init_health_daemon() {
    pros::Task([&]() {
        while (true) {
            health_task();
            pros::delay(20);
        }
    });
}
} // namespace health_daemon
