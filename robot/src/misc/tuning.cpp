/**
 * @file
 * @brief tuning utils
 */

#include "tuning.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
#include "screen/screen.h"
#include <cmath>

void odom_tuning() {
    // units::Pose pose = { 0_in, 0_in };
    // model_manager.setPose(pose);
    // tracker.setPose(pose);

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

        screen::health::set_console_text(
          std::format("if x or y are negative then offets should be flipped.\n"
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
