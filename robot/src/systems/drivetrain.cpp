#include "systems/drivetrain.h"
#include "globals.h"
#include "pros/rtos.hpp"

namespace base {
void driveUpdate() {
    // get left y and right x positions
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot based on these controls
    chassis.arcade(leftY, rightX);
}

void init() {
    pros::Task drivebase_task([] {
        while (true) {
            driveUpdate();
            pros::delay(10);
        }
    });
}
} // namespace base
