#include "apis.h"
//
#include "systems/drivetrain.h"
#include "globals.h"
#include "pros/rtos.hpp"

namespace base {
void driveUpdate() {
    // get left y and right x positions
    int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    int leftPower = throttle + turn;
    int rightPower = throttle - turn;

    left_motors.move(leftPower);
    right_motors.move(rightPower);
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
