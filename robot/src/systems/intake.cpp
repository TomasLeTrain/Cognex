#include "apis.h"
//
#include "auton_globals.h"
#include "blazing/utils.hpp"
#include "globals.h"
#include "globals/device_globals.h"
#include "pros/abstract_motor.hpp"
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

bool is_driver = false;
bool tasks_active = false;

// colorsort sort variables
// bool color_sort_driver = false;
// bool color_sort_auton = false;

// void setAutonColorSort(bool enabled) {
//     color_sort_auton = enabled;
// }
//
// void setDriverColorSort(bool enabled) {
//     color_sort_driver = enabled;
// }

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
std::optional<alliance_t> lower_color;
std::optional<alliance_t> upper_color;
pros::Mutex mutex;

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

std::optional<alliance_t> getLowerColor() {
    return lower_color;
}

std::optional<alliance_t> getUpperColor() {
    return upper_color;
}

void update() {
    lower_color = colorDetected(lower_intake_color_sensor);
    upper_color = colorDetected(upper_intake_color_sensor);
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
    middle_intake_piston_2.set_value(middle == aligned_top);
    bottom_intake_piston.set_value(bottom == up);
}

// sets the top scoring to be blocked
void gate_blocked() {
    set_top(blocking);
}

// sets the top scoring to be passthrough
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

} // namespace pistons

namespace bottom {
pros::Mutex mutex;

Voltage target;

Voltage m_voltage = -1_volt;

// outtaking voltage should be negative!!!
void setTarget(Voltage voltage) {
    m_voltage = voltage;
}

// void set_pct(Voltage new_pct) {
//     std::lock_guard lock(mutex);
//     target = new_pct;
// }
//
// void set_pct(float new_pct) {
//     std::lock_guard lock(mutex);
//     target = new_pct * volt;
// }

// void set_rpm(AngularVelocity new_rpm) {
//     std::lock_guard lock(mutex);
//     target = new_rpm;
// }
//
// void set_rpm_pct(float new_vel_pct) {
//     std::lock_guard lock(mutex);
//     target = new_vel_pct * 600_rpm;
// }
//
// void set_antijam(bool active) {
//     std::lock_guard lock(mutex);
//     antijam_active = active;
// }

// void set_outtake_antijam(bool active) {
//     std::lock_guard lock(mutex);
//     outtake_antijam = active;
// }

// directly updates hardware
// should only be used by update function
void hardware_move_pct(Voltage pct) {
    bottom_motor.move_voltage(12 * to_mvolt(pct));
}

// update can be blocking if antijam or color sort are active
// while blocking it also locks the mutex
void update() {
    std::lock_guard lock(mutex);

    // entirely determined by the lever state?
    // bool bottom_jammed = motorJammed(bottom_motor);

    auto lever_state = lever::getTarget();
    float lever_position = lever::getLeverPosition();

    if (lever_position > 0.2) {
        // lever is up, no case in which we want to be intaking
        hardware_move_pct(-0.5_volt);
    } else {
        hardware_move_pct(m_voltage);
    }

    //
    // if (antijam_active && bottom_jammed) {
    //     // move bottom at 0 speed
    //     if (outtake_antijam) {
    //         hardware_move_pct(-1.0_volt);
    //         // scoring antijam action
    //         pros::delay(to_msec(antijam_outtake_timeout));
    //     } else {
    //         hardware_move_pct(0.0_volt);
    //         // scoring antijam action
    //         pros::delay(to_msec(antijam_timeout));
    //     }
    //
    //     // afterwards move as normal, give it time to setttle
    //     Time start_move_normal = now();
    //     while (!timeoutDone(settle_time, start_move_normal)) {
    //         if (std::holds_alternative<Voltage>(target) &&
    //             // want to stop intake, stop immediately
    //             // slever means to stop
    //             units::abs(get<Voltage>(target).internal()) <= 0.01) {
    //             break;
    //         }
    //         hardware_update();
    //         pros::delay(10);
    //     }
    //
    //     // afterwards goes through update again, if still jammed then
    //     // antijams again
    // } else {
    //     // no antijam active, move like normal
    //     hardware_update();
    // }
}

} // namespace bottom

namespace lever {
pros::Mutex mutex;

LeverVelocityProfile::LeverVelocityProfile(AngularVelocity v0,
                                           AngularVelocity v1)
    : v0(v0),
      v1(v1) {}

// returns the desired target velocity at angle theta in range [0,1]
AngularVelocity LeverVelocityProfile::velocity(float theta) {
    return v0 + (v1 - v0) * theta;
}

// determines when the profile is finished with a motion
// if finished, returns a discrete lever state to go back to
// TODO: or maybe a target theta?
std::optional<DiscreteLeverState> LeverVelocityProfile::finished(float theta) {
    if (theta >= 0.95) {
        return going_up;
    } else {
        return std::nullopt;
    }
}

// make sure lever resets at the beginning of program
std::variant<Voltage, DiscreteLeverState, float, LeverVelocityProfile>
  m_target = DiscreteLeverState::going_down;

// lever_position = motor_position * position_to_theta_mult

bool has_red_cart = false;
float red_card_mult = has_red_cart ? (1 / 2.f) : 1;

float position_to_theta_mult = (1 / 0.842) * red_card_mult;
float zero_motor_position;
float lever_position;

// TODO: move to be modifiable
Voltage action_voltage = 1_volt;

void setActionVoltage(Voltage actionVoltage) {
    action_voltage = actionVoltage;
}

void setTarget(
  std::variant<Voltage, DiscreteLeverState, float, LeverVelocityProfile>
    target) {
    m_target = target;
}

std::variant<Voltage, DiscreteLeverState, float, LeverVelocityProfile>
getTarget() {
    return m_target;
}

float getLeverPosition() {
    return lever_position;
}

void hardware_update(Voltage voltage) {
    lever_motor.move_voltage(12 * to_mvolt(voltage));
}

// void controller_init() {
//     // should get run once before running update?
//     zero_motor_position = lever_motor.get_position();
// }

void controller_update() {
    lever_position = (lever_motor.get_position() - zero_motor_position) *
                     position_to_theta_mult;

    const bool stalling = motorJammed(lever_motor);

    if (std::holds_alternative<Voltage>(m_target)) {
        // move with the desired voltage
        hardware_update(std::get<Voltage>(m_target));
    }
    if (std::holds_alternative<DiscreteLeverState>(m_target)) {
        auto target_state = std::get<DiscreteLeverState>(m_target);

        Voltage applied_voltage;
        if (target_state == down || target_state == up) {
            // not moving, keep 0 voltage with motor hold
            applied_voltage = 0_volt;
        } else if (target_state == going_down) {
            // going down with action_voltage
            applied_voltage = -action_voltage;

            // when close enough to the bottom switch to wanting to reset
            if (lever_position < 0.1) {
                m_target = DiscreteLeverState::position_reset;
            }
        } else if (target_state == going_up) {
            // go up with action voltage
            applied_voltage = action_voltage;

            // condition for stopping going up is stalling while close to
            // the target end
            if (lever_position > 0.93 && stalling) {
                // lever fully up?
                m_target = up;
            }
        } else if (target_state == position_reset) {
            // TODO: determine
            applied_voltage = -0.5_volt;
            if (stalling) {
                // reset motor position and switch to being down
                zero_motor_position = lever_motor.get_position();
                m_target = down;
            }
        }

        hardware_update(applied_voltage);
    } else if (std::holds_alternative<float>(m_target)) {
        // use some sort of control to move lever to target, possibly pid?

        float position_target = std::get<float>(m_target);

    } else if (std::holds_alternative<LeverVelocityProfile>(m_target)) {
        // follow profile with some sort of way
        LeverVelocityProfile profile = std::get<LeverVelocityProfile>(m_target);

        auto desired_vel = profile.velocity(lever_position);

        // TODO: follow desired vel with some controller
        auto finished = profile.finished(lever_position);
        if (finished.has_value()) {
            // switch to the desired discrete state
            m_target = finished.value();
        }
    }
}

bool target_up = false;
Time score_start_time = 0_sec;

// should not really be blocking
void update() {
    std::lock_guard lock(mutex);

    lever_motor.set_brake_mode(pros::MotorBrake::hold);

    controller_update();
}

} // namespace lever

//
// common intake states are defined here
//

// sets the target of the lever such that eventually it reaches the down state
// and stays there
void continuous_lever_down() {
    // go max voltage down
    lever::setActionVoltage(1.0_volt);

    auto current_lever_target = lever::getTarget();
    if (std::holds_alternative<lever::DiscreteLeverState>(
          current_lever_target) &&
        (std::get<lever::DiscreteLeverState>(current_lever_target) ==
           lever::down ||
         std::get<lever::DiscreteLeverState>(current_lever_target) ==
           lever::position_reset)) {
        // already down, change nothing
    } else {
        lever::setTarget(lever::going_down);
    }
}

// sets the target of the lever such that eventually it reaches the down state
// and stays there
void continuous_lever_up(Voltage actionVoltage) {
    // go max voltage down
    lever::setActionVoltage(actionVoltage);

    auto current_lever_target = lever::getTarget();
    if (std::holds_alternative<lever::DiscreteLeverState>(
          current_lever_target) &&
        std::get<lever::DiscreteLeverState>(current_lever_target) ==
          lever::up) {
        // already down, change nothing
    } else {
        lever::setTarget(lever::going_up);
    }
}

//
// only pauses motors, does not change piston states
void motors_disabled() {
    continuous_lever_down();
    bottom::setTarget(0_volt);
}

void in() {
    continuous_lever_down();
    bottom::setTarget(1_volt);

    // does not set alignment
    pistons::align_top();
    pistons::gate_blocked();
    pistons::intake_down();
}

void out() {
    continuous_lever_down();
    bottom::setTarget(-1_volt);

    // align top so all blocks come out
    pistons::align_top();
    pistons::gate_blocked();
    pistons::intake_down();
}

void score_long(Voltage actionVoltage) {
    continuous_lever_up(actionVoltage);
    bottom::setTarget(1_volt);

    pistons::align_top();
    pistons::gate_scoring();
    pistons::intake_down();
}

// defaults:
void score_middle(Voltage actionVoltage) {
    continuous_lever_up(actionVoltage);
    bottom::setTarget(1_volt);

    pistons::align_middle();
    pistons::gate_scoring();
    pistons::intake_down();
}

// defaults: bottom_speed = -0.5,  lever_speed = -1.0
void score_bottom() {
    continuous_lever_down();
    bottom::setTarget(-0.5_volt);

    pistons::align_top();
    pistons::gate_blocked();
    pistons::intake_up();
}

namespace driver {
void update() {
    if (!is_driver) return;
    // update states based on driver input
    bool driver_intake = controller.get_digital(controls::L1);

    bool score_bottom_height = controller.get_digital(controls::L2);
    bool score_middle_height = controller.get_digital(controls::R2);
    bool scoreLong = controller.get_digital(controls::R1);

    // bool kill_color_sort = controller.get_digital_new_press(controls::LEFT);

    bool unjam = controller.get_digital(controls::X);

    // one time kill switch
    // if (color_sort_driver == true && kill_color_sort) {
    //     color_sort_driver = false;
    // }

    if (unjam) {
        out();
    }

    else if (driver_intake) {
        in();
    }

    else if (score_middle_height) {
        score_middle(0.4_volt);
    }

    else if (score_bottom_height) {
        score_bottom();
    }

    else if (scoreLong) {
        score_long(0.8_volt);
    } else {
        motors_disabled();
        // alignment is not set when disabled
        pistons::gate_blocked();
        pistons::intake_down();
    }
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

    setup_color_sensor(lower_intake_color_sensor);
    setup_color_sensor(upper_intake_color_sensor);

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
