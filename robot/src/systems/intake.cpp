#include "apis.h"
//
#include "auton_globals.h"
#include "blazing/utils.hpp"
#include "globals.h"
#include "globals/device_globals.h"
#include "lyfast/vel_controller.hpp"
#include "pros/device.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "systems/intake.h"
#include "systems/piston.h"
#include "units/Angle.hpp"
#include "units/units.hpp"
#include <map>
#include <mutex>
#include <optional>
#include <variant>

namespace intake {

class IntakeVelocityController {
  private:
    pros::Motor* m_motor;
    lyfast::SimpleVelocityControllerParams<AngularVelocity> m_params;
    double m_alpha = 0.9;

    std::variant<Voltage, AngularVelocity> m_target;

    std::optional<AngularVelocity> last_error = std::nullopt;
    std::optional<AngularVelocity> last_measurement = std::nullopt;
    Angle integral = 0.0 * deg;

    AngularVelocity getMeasurement() {
        auto curr_measurement = m_motor->get_actual_velocity() * rpm;
        AngularVelocity result;
        if (last_measurement.has_value()) {
            // low pass filter the velocity
            result =
              // TODO: depends on how much time from last measurement?
              last_measurement.value() * (1 - m_alpha) +
              m_alpha * curr_measurement;
        } else {
            result = curr_measurement;
        }

        last_measurement = curr_measurement;

        return result;
    }

    void moveVoltage(Voltage target_voltage) {
        m_motor->move_voltage(to_mvolt(target_voltage) * 12);
    }

  public:
    IntakeVelocityController(
      pros::Motor* motor,
      lyfast::SimpleVelocityControllerParams<AngularVelocity> params,
      double alpha = 0.9)
        : m_motor(motor),
          m_params(params),
          m_alpha(alpha) {}

    void setTarget(std::variant<Voltage, AngularVelocity> target) {
        m_target = target;
    }

    void update(Time duration) {
        if (std::holds_alternative<Voltage>(m_target)) {
            // useful for full speed commands
            moveVoltage(std::get<Voltage>(m_target));
            return;
        }
        AngularVelocity target = std::get<AngularVelocity>(m_target);

        m_motor->move_velocity(to_rpm(target));

        return;

        // else we are using velocity control
        // TODO: what to do if motor unplugs??

        AngularVelocity measurement = getMeasurement();

        AngularVelocity error = target - measurement;

        Angle current_integral = integral;

        if (last_error)
            // use trapezoidal approximation
            current_integral += (error + *last_error) * duration / 2.0;
        else
            // use Riemann sum approximation
            current_integral += error * duration;

        Voltage result {
            // kv
            target * m_params.Kv +
              // ks
              units::sgn(target) * m_params.Ks +
              // kp
              m_params.Kp * error +
              // ki
              m_params.Ki * current_integral,
        };

        if (
          // currently saturating
          units::abs(result) >= m_params.max_output &&
          // output going in direct of error
          units::sgn(error) == units::sgn(result)) {
            // clamping, slever integral windup
            // no need to update integral to current integral
        } else {
            // not saturating, update integral
            integral = current_integral;
        }

        result =
          units::clamp(result, -m_params.max_output, m_params.max_output);

        last_error = error;

        moveVoltage(result);
    }
};

bool is_driver = false;
bool tasks_active = false;

// colorsort sort variables
bool color_sort_driver = false;
bool color_sort_auton = false;

void setAutonColorSort(bool enabled) {
    color_sort_auton = enabled;
}

void setDriverColorSort(bool enabled) {
    color_sort_driver = enabled;
}

bool motorJammed(pros::Motor& motor) {
    float thresh_vel = 1;

    return (std::abs(motor.get_voltage()) > 10 && motor.get_torque() > 0.1 &&
            std::fabs(motor.get_actual_velocity()) < thresh_vel);
}

bool motorSlowed(pros::Motor& motor) {
    float thresh_vel = 100;

    return (std::abs(motor.get_voltage()) > 10 && motor.get_torque() > 0.1 &&
            std::fabs(motor.get_actual_velocity()) < thresh_vel);
}

namespace colors {
std::optional<alliance_t> middle_color;

std::optional<alliance_t> colorDetected(pros::Optical& sensor) {
    // detect color from color sensor
    std::optional<alliance_t> result = std::nullopt;

    // just say no balls are being detected
    if (!sensor.is_installed()) return result;

    double color_sensor_hue = sensor.get_hue();

    // intake senses something
    if (sensor.get_proximity() > 250) {
        if (color_sensor_hue > 300 || color_sensor_hue < 100)
            result = alliance_t::red;
        else if (color_sensor_hue > 120 && color_sensor_hue <= 300)
            result = alliance_t::blue;
    }

    return result;
}

std::optional<alliance_t> getMiddleColor() {
    return middle_color;
}

void update() {
    middle_color = colorDetected(middle_intake_color_sensor);
}
}; // namespace colors

namespace pistons {
pros::Mutex mutex;

top_state_t top = blocking;
middle_state_t middle = aligned_top;
bool bottom = false;

void set_top(top_state_t top_state) {
    std::lock_guard lock(mutex);
    top = top_state;
}

void set_middle(middle_state_t middle_state) {
    std::lock_guard lock(mutex);
    middle = middle_state;
}

void set_bottom(bottom_state_t bottom_state) {
    std::lock_guard lock(mutex);
    bottom = bottom_state;
}

void update() {
    std::lock_guard lock(mutex);
    gate_intake_piston.set_value(top == passthrough);
    middle_intake_piston.set_value(middle == aligned_middle);
    bottom_intake_piston.set_value(bottom == up);
}

// sets the lever scoring to be blocked
void gate_blocked() {
    set_top(blocking);
}

// sets the lever scoring to be passthrough
void gate_scoring() {
    set_top(passthrough);
}

void align_top() {
    set_middle(aligned_top);
}

void align_middle() {
    set_middle(aligned_middle);
}

void intake_up() {
    set_bottom(up);
}

void intake_down() {
    set_bottom(down);
}

// helper functions for various configurations
void blocked_lever_aligned() {
    gate_blocked();
    align_top();
}

void blocked_middle_aligned() {
    gate_blocked();
    align_middle();
}

void score_lever_aligned() {
    gate_scoring();
    align_top();
}

void score_middle_aligned() {
    gate_scoring();
    align_middle();
}

} // namespace pistons

namespace bottom {
pros::Mutex mutex;

lyfast::SimpleVelocityControllerParams<AngularVelocity> vel_controller_params {
    .Kv = (1_volt / 600_rpm),
    // not really used
    .Ka = 0.0 * volt / radps2,
    .Ks = 0.02 * volt,
    // .Kp = 1_volt / 100_rpm,
    .Kp = 0.2_volt / 100_rpm,
    .Ki = 1.0 * volt / rad,
};

IntakeVelocityController controller(&bottom_motor, vel_controller_params);

std::variant<Voltage, AngularVelocity> target;
// Voltage pct;
// AngularVelocity target_rpm;

bool antijam_active = true;

// amount of time we antijam
Time antijam_timeout = 100_msec;

Time antijam_outtake_timeout = 100_msec;

// allows intake the settle right after antijamming, possibly avoids triggering
// antijam immediately afterwards even if jam is cleared
Time settle_time = 500_msec;

bool outtake_antijam = false;

void set_pct(Voltage new_pct) {
    std::lock_guard lock(mutex);
    target = new_pct;
}

void set_pct(float new_pct) {
    std::lock_guard lock(mutex);
    target = new_pct * volt;
}

void set_rpm(AngularVelocity new_rpm) {
    std::lock_guard lock(mutex);
    target = new_rpm;
}

void set_rpm_pct(float new_vel_pct) {
    std::lock_guard lock(mutex);
    target = new_vel_pct * 600_rpm;
}

void set_antijam(bool active) {
    std::lock_guard lock(mutex);
    antijam_active = active;
}

void set_outtake_antijam(bool active) {
    std::lock_guard lock(mutex);
    outtake_antijam = active;
}

// directly updates hardware
// should only be used by update function
void hardware_move_pct(Voltage pct) {
    bottom_motor.move_voltage(12 * to_mvolt(pct));
}

void hardware_update() {
    controller.setTarget(target);

    // TODO: make sure its actually this update rate
    controller.update(10_msec);
}

// update can be blocking if antijam or color sort are active
// while blocking it also locks the mutex
void update() {
    std::lock_guard lock(mutex);

    bool bottom_jammed = motorJammed(bottom_motor);

    if (antijam_active && bottom_jammed) {
        // move bottom at 0 speed
        if (outtake_antijam) {
            hardware_move_pct(-1.0_volt);
            // scoring antijam action
            pros::delay(to_msec(antijam_outtake_timeout));
        } else {
            hardware_move_pct(0.0_volt);
            // scoring antijam action
            pros::delay(to_msec(antijam_timeout));
        }

        // afterwards move as normal, give it time to setttle
        Time start_move_normal = now();
        while (!timeoutDone(settle_time, start_move_normal)) {
            if (std::holds_alternative<Voltage>(target) &&
                // want to slever intake, stop immediately
                units::abs(get<Voltage>(target).internal()) <= 0.01) {
                break;
            }
            hardware_update();
            pros::delay(10);
        }

        // afterwards goes through update again, if still jammed then
        // antijams again
    } else {
        // no antijam active, move like normal
        hardware_update();
    }
}

} // namespace bottom

namespace lever {
pros::Mutex mutex;

// lyfast::SimpleVelocityControllerParams<AngularVelocity> vel_controller_params
// {
//     .Kv = (1_volt / 600_rpm),
//     // not really used
//     .Ka = 0.0 * volt / radps2,
//     .Ks = 0.02 * volt,
//     // .Kp = 1_volt / 100_rpm,
//     .Kp = 0.2_volt / 100_rpm,
//     .Ki = 1.0 * volt / rad,
// };

// IntakeVelocityController controller(&lever_motor, vel_controller_params);

// float close_threshold = 10.f / 100.f;
// float position;
// float target_position = 0;

bool target_up = false;

// motor position when lever is down. should get recalibrated with every
// reset
// float motor_position_offset = 0;
//
// // turns the whole motor range into a number from 0-1
// float motor_position_factor = 1;
//
// // while true hardware update does nothing
// bool manual_voltage_control = false;
//
// Voltage hardware_target;
// // AngularVelocity target_speed;
//
// bool antijam_active = false;

// latest time since we started scoring
// used to slever antijam from running for the first 200_msec of scoring
// bool scoring
bool m_is_scoring = false;
Time score_start_time = 0_sec;

// void set_target(float new_target) {
//     std::lock_guard lock(mutex);
//     // target_position = new_target;
// }

void set_target(bool up) {
    std::lock_guard lock(mutex);
    if (!target_up && up) score_start_time = now();
    target_up = up;
}

void set_antijam(bool active) {
    std::lock_guard lock(mutex);
    // antijam_active = active;
}

// update scoring status, used by antijam
// updating does not affect antijam active state
// void set_scoring(bool is_scoring) {
//     std::lock_guard lock(mutex);
//     m_is_scoring = is_scoring;
//     if (m_is_scoring) score_start_time = now();
// }

// resets position offset - MAKE SURE LEVER IS DOWN!
// void resetPositionOffset() {
//     motor_position_offset = lever_motor.get_position();
// }
//
// void setManualVoltageControl(bool enabled) {
//     std::lock_guard lock(mutex);
//
//     // stop any logic of controlling lever
//     manual_voltage_control = enabled;
// }

// make sure manual voltage control is enabled for this to take effect
// void setHardwareTarget(Voltage voltage_target) {
//     std::lock_guard lock(mutex);
//
//     // stop any logic of controlling lever
//     hardware_target = voltage_target;
// }

// should be called to reset the lever position. useful every time lever is down
// void positionOffsetRoutine() {
//     // stop any logic of controlling lever
//     setManualVoltageControl(true);
//
//     // purposely stall the motor
//     setHardwareTarget(-0.5_volt);
//
//     // waits until it stalls
//     while (!motorJammed(lever_motor)) {
//         pros::delay(10);
//     }
//
//     // reset the position offset
//     resetPositionOffset();
//
//     // go back to auto controlled
//     setManualVoltageControl(false);
// }

// void updatePosition() {
//     // get new position
//     float motor_reported_position = lever_motor.get_position();
//
//     position =
//       motor_position_factor * (motor_reported_position -
//       motor_position_offset);
// }
//
// void hardware_update() {
//     lever_motor.move_voltage(12 * to_mvolt(hardware_target));
// }

// update can be blocking if antijam or color sort are active
// while blocking it also locks the mutex
void update() {
    std::lock_guard lock(mutex);

    // bool jam = motorJammed(lever_motor);

    if (target_up) {
        lever_motor.move_voltage(12000);
    } else {
        if (timeoutDone(1_sec, score_start_time)) {
            lever_motor.move_voltage(0);
        } else {
            lever_motor.move_voltage(-12000);
        }
    }

    // if not antijam then hardware updates
    // updatePosition();
    //
    // if (!manual_voltage_control) {
    //     float error = target_position - position;
    //
    //     if (units::abs(error) < close_threshold) {
    //         // close enough, give minimal voltage in direction of error
    //         hardware_target = 0.2 * units::sgn(error) * volt;
    //         // else use simple bang bang controller
    //     } else {
    //         hardware_target = units::sgn(error) * volt;
    //     }
    // }
    //
    // hardware_update();
}

} // namespace lever

// some more helper functions to make declaring states easier
//
// sets pct for both intake motors
void set_pct(auto pct) {
    bottom::set_pct(pct);
    // lever::set_pct(pct);
}

// sets pct for both intake motors
void set_pct(auto bottom, auto lever) {
    bottom::set_pct(bottom);
    // lever::set_pct(top);
}

// sets antijam for both
void set_antijam(bool active) {
    lever::set_antijam(true);
    bottom::set_antijam(true);
}

// sets antijam for both
void set_antijam(bool bottom_active, bool lever_active) {
    lever::set_antijam(true);
    bottom::set_antijam(true);
}

//
// common intake states are defined here
//
// only pauses motors, does not change piston states
void motors_disabled() {
    set_pct(0.0);
    lever::set_target(false);
}

void in() {
    set_pct(1.0);
    lever::set_target(false);
    // does not set alignment
    pistons::gate_blocked();
    pistons::intake_down();
    pistons::align_top();
}

void intake_middle_balls(float bottom_speed, float lever_speed) {
    set_pct(bottom_speed, lever_speed);

    pistons::gate_blocked();
    pistons::intake_down();
    pistons::align_top();
    lever::set_target(false);
}

void out() {
    set_pct(-1.0);
    lever::set_target(false);
    // does not set alignment
    pistons::gate_blocked();
    pistons::intake_down();
}

void score_long_no_outtake(float bottom_speed, float lever_speed) {
    set_pct(bottom_speed, lever_speed);

    pistons::score_lever_aligned();
    pistons::intake_down();
    lever::set_target(true);
}

void score_long(float bottom_speed, float lever_speed) {
    // outtake first
    set_pct(0, -1);
    pros::delay(100);

    // score
    score_long_no_outtake(bottom_speed, lever_speed);
}

// defaults:
// score_middle -> bottom_speed = 1.0, lever_speed = 0.3
void score_middle(float bottom_speed, float lever_speed) {
    // set_pct(bottom_speed, lever_speed);
    bottom::set_pct(bottom_speed);
    // use speed for the lever
    lever::set_target(true);

    pistons::score_middle_aligned();
    pistons::intake_down();
}

void score_middle_slow() {
    // score middle even slower
    score_middle(1.0, 0.25);
}

// defaults: bottom_speed = -0.5,  lever_speed = -1.0
void score_bottom(float bottom_speed, float lever_speed) {
    // set_pct(bottom_speed, lever_speed);
    // set_pct(bottom_speed, lever_speed);
    bottom::set_rpm_pct(bottom_speed);
    lever::set_target(false);
    // use speed for the lever
    // lever::set_pct(top_speed);

    // only updates bottom piston, no need to update others
    pistons::intake_up();
}

void score_bottom_slow() {
    score_middle(-0.3, -1.0);
}

namespace driver {
bool was_scoring = false;

void update() {
    if (!is_driver) return;

    // CANNOT RETURN AFTER THIS POINT FOR WAS SCORING

    // update states based on driver input
    bool driver_intake = controller.get_digital(controls::L1);

    bool score_bottom_height = controller.get_digital(controls::L2);
    bool score_middle_height = controller.get_digital(controls::R2);
    bool scoreLong = controller.get_digital(controls::R1);

    bool kill_color_sort = controller.get_digital_new_press(controls::LEFT);

    bool unjam = controller.get_digital(controls::X);

    // ONLY USED IN SKILLS
    // bool score_slow = controller.get_digital(controls::RIGHT_SHIFT);

    // one time kill switch
    if (color_sort_driver == true && kill_color_sort) {
        color_sort_driver = false;
    }

    else if (unjam) {
        out();
    }

    else if (driver_intake) {
        in();
    }

    else if (score_middle_height) {
        score_middle(1.0, 0.7);
    }

    else if (score_bottom_height) {
        score_bottom(-0.6, -1.0);
    }

    else if (scoreLong) {
        if (was_scoring) {
            // run with no outtake?
            // do nothing?
        } else {
            // just do normal
            score_long();

            was_scoring = true;
        }
    } else {
        motors_disabled();
        pistons::gate_blocked();
        pistons::intake_down();
    }

    // not scoring anymore
    if (was_scoring && !scoreLong) was_scoring = false;
}
} // namespace driver

// initializes pistons, lever, and bottom systems
void init(bool driver) {
    is_driver = driver;

    // set some hardware related options
    auto setup_color_sensor = [](pros::Optical& sensor) {
        if (sensor.is_installed()) {
            sensor.disable_gesture();
            sensor.set_integration_time(10);
            sensor.set_led_pwm(255);
        }
    };

    setup_color_sensor(middle_intake_color_sensor);

    // don't make another task
    if (tasks_active) return;

    // run any code here that should only occur once

    pros::Task piston_task(
      [] {
          while (true) {
              pistons::update();
              pros::delay(10);
          }
      },
      "intake pistons task");

    pros::Task bottom_motor_task(
      [] {
          while (true) {
              bottom::update();
              pros::delay(10);
          }
      },
      "bottom motor task");

    pros::Task lever_motor_task(
      [] {
          while (true) {
              lever::update();
              pros::delay(10);
          }
      },
      "lever motor task");

    pros::Task simple_tasks_intake(
      [] {
          while (true) {
              colors::update();
              driver::update();
              pros::delay(10);
          }
      },
      "simple intake tasks");

    tasks_active = true;
}

} // namespace intake
