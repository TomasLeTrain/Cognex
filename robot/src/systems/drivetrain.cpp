#include "apis.h"
//
#include "globals.h"
#include "globals/blazing_globals.h"
#include "pros/rtos.hpp"
#include "systems/drivetrain.h"
#include <cmath>

namespace base {
bool started = false;

void driveUpdate() {
    // get left y and right x positions

    // float linear_deadband = 3;
    // float linear_curveGain = 1.019;
    // float linear_minOutput = 10;

    float angular_deadband = 3;
    // float angular_curveGain = 1.019;
    float angular_curveGain = 1.001;
    float angular_minOutput = 10;

    bool use_expo = true;

    auto curve_func = [&](float input,
                          float deadband,
                          float curveGain,
                          float minOutput) -> float {
        if (fabs(input) <= deadband) return 0;
        const float g = fabs(input) - deadband;
        const float g127 = 127 - deadband;
        const float i = pow(curveGain, g - 127) * g * units::sgn(input);
        const float i127 = pow(curveGain, g127 - 127) * g127;

        return (127.0 - minOutput) / (127) * i * 127 / i127 +
               minOutput * units::sgn(input);
    };

    int throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (use_expo) {
        // throttle = std::round(curve_func(throttle, linear_deadband,
        //                                  linear_curveGain,
        //                                  linear_minOutput));

        turn = std::round(curve_func(turn,
                                     angular_deadband,
                                     angular_curveGain,
                                     angular_minOutput));
    }

    // desaturate?

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
