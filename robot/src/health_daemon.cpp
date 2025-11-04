#include "apis.h"
//

#include "globals.h"
#include "globals/config.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
#include "health_daemon.h"
#include "pros/device.hpp"
#include "screen/screen.h"
#include "vexmaps/api.hpp"

namespace health_daemon {
// avoids spamming notifications
// every error only every produces one notification
std::vector<bool> port_dc(22, false);
bool imu_invalid = false;

bool vexmaps_tracker_inf = false;
bool blazing_tracker_inf = false;
bool blazing_tracker_heading_inf = false;

bool map_reader_unavailable = false;

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
            auto zero_indexed_port = abs(port) - 1;
            bool installed =
              pros::DeviceType::motor ==
              (pros::DeviceType)pros::c::registry_get_plugged_type(
                zero_indexed_port);
            if (!installed && !port_dc[port]) {
                port_dc[port] = true;

                screen::health::add_notification(
                  std::format("Port {}: Motor unplugged!", port),
                  std::format(
                    "Motor on {} motor group.\nMake sure this is not critical!",
                    motor_group_name),
                  screen::health::warn);
            }
        }
    };

    check_motor_group(left_motors, "left");
    check_motor_group(right_motors, "right");

    auto process_device_dc =
      [&](auto& device,
          std::string device_name,
          screen::health::notification_severity_t severity =
            screen::health::warn,
          std::string custom_mesg = "Make sure its not critical!") {
          if (!device.is_installed() && dc_not_processed(device)) {
              update_dc(device);

              screen::health::add_notification(
                std::format("Port {}: {} unplugged!",
                            device.get_port(),
                            device_name),
                custom_mesg,
                severity);
          }
      };

    // check imu dc / invalid heading
    process_device_dc(imu,
                      "IMU",
                      screen::health::critical,
                      "This is likely very bad!");

    if (!imu.is_calibrating() && !std::isfinite(imu.get_rotation()) &&
        !imu_invalid) {
        imu_invalid = true;
        screen::health::add_notification(
          std::format("Port {}: IMU returns INF!", imu.get_port()),
          "This is likely very bad!",
          screen::health::critical);
    }

    // check rotation sensors dc
    process_device_dc(forwards_odom_rotation,
                      "Forwards rotation",
                      screen::health::critical,
                      "This is likely very bad!");
    process_device_dc(sideways_odom_rotation,
                      "Sideways rotation",
                      screen::health::critical,
                      "This is likely very bad!");

    process_device_dc(front_distance, "Front distance");
    process_device_dc(back_distance, "Back distance");
    process_device_dc(left_distance, "Left distance");
    process_device_dc(right_distance, "Right distance");

    process_device_dc(bottom_motor, "Bottom Intake", screen::health::critical);
    process_device_dc(top_motor, "Top Intake", screen::health::critical);

    // now check tracking subsystems

    if (auto curr_pose = model_manager.getPose();
        (!isfinite(curr_pose.x.internal()) ||
         !isfinite(curr_pose.y.internal()) ||
         !isfinite(curr_pose.orientation.internal())) &&
        !vexmaps_tracker_inf) {
        vexmaps_tracker_inf = true;
        screen::health::add_notification("model being used is INF!",
                                         "Make sure this isn't critical!",
                                         screen::health::warn);
    }

    if (auto curr_position = tracker.getPosition();
        (!isfinite(curr_position.x.internal()) ||
         !isfinite(curr_position.y.internal())) &&
        !blazing_tracker_inf) {
        blazing_tracker_inf = true;
        screen::health::add_notification("Blazing Tracker is INF!",
                                         "Make sure this isn't critical!",
                                         screen::health::warn);
    }
    if (!isfinite(tracker.getAngle().internal()) &&
        !blazing_tracker_heading_inf) {
        blazing_tracker_heading_inf = true;
        screen::health::add_notification("Blazing Tracker theta is INF!",
                                         "Make sure this isn't critical!",
                                         screen::health::warn);
    }
    if (!map_reader.mapAvailable() &&
        !map_reader_unavailable) {
        map_reader_unavailable = true;
        screen::health::add_notification("Map was not read!",
                                         "Make sure this isn't critical!",
                                         screen::health::warn);
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
