#include "apis.h"
//
#include "globals.h"
#include "globals/blazing_globals.h"
#include "pros/rtos.hpp"
#include "systems/drivetrain.h"

namespace base {
bool started = false;

void driveUpdate() {
    // get left y and right x positions
    // int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    // int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    //
    // int leftPower = throttle + turn;
    // int rightPower = throttle - turn;
    //
    // left_motors.move(leftPower);
    // right_motors.move(rightPower);

    Voltage dir = from_volt(
      controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0);
    Voltage turn = -from_volt(
      controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0);

    dir = driver_linear_slew.linear_slew.apply(dir, 10_msec);

    drivetrain.moveArcade(dir, turn);
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
