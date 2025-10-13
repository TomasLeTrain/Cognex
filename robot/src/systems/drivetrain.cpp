#include "apis.h"
//
#include "globals.h"
#include "pros/rtos.hpp"
#include "systems/drivetrain.h"

namespace base {
bool started = false;

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
    if (started) return;

    pros::Task drivebase_task([] {
        while (true) {
            driveUpdate();
            pros::delay(10);
        }
    });

    started = true;
}
} // namespace base
