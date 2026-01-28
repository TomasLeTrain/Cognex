#include "apis.h"
//
#include "auton_globals.h"
#include "blazing/utils.hpp"
#include "globals.h"
#include "globals/device_globals.h"
#include "pros/rtos.hpp"
#include "systems/new_intake.h"
#include "systems/piston.h"
#include "units/units.hpp"
#include <map>
#include <mutex>
#include <optional>

namespace new_intake {
bool is_driver = false;
bool tasks_active = false;

// colorsort sort variables
bool color_sort_driver = false;

bool motorJammed(pros::Motor& motor) {
    double thresh_vel = 1;

    return (std::abs(motor.get_voltage()) > 10 && motor.get_torque() > 0.1 &&
            std::fabs(motor.get_actual_velocity()) < thresh_vel);
}

bool motorSlowed(pros::Motor& motor) {
    double thresh_vel = 100;

    return (std::abs(motor.get_voltage()) > 10 && motor.get_torque() > 0.1 &&
            std::fabs(motor.get_actual_velocity()) < thresh_vel);
}

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
    top_intake_piston.set_value(top == blocking);
    middle_intake_piston.set_value(middle == aligned_middle);
    bottom_intake_piston.set_value(bottom == up);
}

// sets the top scoring to be blocked
void scoring_blocked() {
    set_top(blocking);
}

// sets the top scoring to be passthrough
void scoring_passthrough() {
    set_top(blocking);
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
void blocked_top_aligned() {
    scoring_blocked();
    align_top();
}

void blocked_middle_aligned() {
    scoring_blocked();
    align_middle();
}

void score_top_aligned() {
    scoring_passthrough();
    align_top();
}

void score_middle_aligned() {
    scoring_passthrough();
    align_middle();
}

} // namespace pistons

namespace bottom {
pros::Mutex mutex;

Voltage pct;
bool antijam_active = false;

// amount of time we antijam
Time antijam_timeout = 100_msec;

// allows intake the settle right after antijamming, possibly avoids triggering
// antijam immediately afterwards even if jam is cleared
Time settle_time = 500_msec;

void set_pct(Voltage new_pct) {
    std::lock_guard lock(mutex);
    pct = new_pct;
}

void set_pct(float new_pct) {
    set_pct(new_pct * volt);
}

void set_antijam(bool active) {
    std::lock_guard lock(mutex);
    antijam_active = active;
}

// directly updates hardware
// should only be used by update function
void hardware_move_pct(Voltage pct) {
    bottom_motor.move_voltage(12 * to_mvolt(pct));
}

// update can be blocking if antijam or color sort are active
// while blocking it also locks the mutex
void update() {
    std::lock_guard lock(mutex);

    bool bottom_jammed = motorJammed(bottom_motor);

    if (antijam_active && bottom_jammed) {
        // move bottom at 0 speed
        hardware_move_pct(0.0_volt);

        // scoring antijam action
        pros::delay(to_msec(antijam_timeout));

        // afterwards move as normal
        hardware_move_pct(pct);

        // give time to settle
        pros::delay(to_msec(settle_time));

        // afterwards goes through update again, if still jammed then
        // antijams again
    } else {
        // no antijam active, move like normal
        hardware_move_pct(pct);
    }
}

} // namespace bottom

namespace top {
pros::Mutex mutex;

Voltage pct;
bool antijam_active = false;

// latest time since we started scoring
// used to stop antijam from running for the first 200_msec of scoring
// bool scoring
bool scoring_antijam = false;
Time score_start_time = 0_sec;

// initial timeout that allows hood to raise up before antijamming
Time initial_timeout = 200_msec;

// amount of time we antijam
Time antijam_timeout = 100_msec;

// allows intake the settle right after antijamming, possibly avoids triggering
// antijam immediately afterwards even if jam is cleared
Time settle_time = 130_msec;

void set_pct(Voltage new_pct) {
    std::lock_guard lock(mutex);
    pct = new_pct;
}

void set_pct(float new_pct) {
    set_pct(new_pct * volt);
}

void set_antijam(bool active) {
    std::lock_guard lock(mutex);
    antijam_active = active;
}

// update scoring status, used by antijam
// updating does not affect antijam active state
void set_scoring(bool is_scoring) {
    std::lock_guard lock(mutex);
    scoring_antijam = is_scoring;
    if (scoring_antijam) score_start_time = now();
}

// directly updates hardware
// should only be used by update function
void hardware_move_pct(Voltage pct) {
    top_motor.move_voltage(12 * to_mvolt(pct));
}

// update can be blocking if antijam or color sort are active
// while blocking it also locks the mutex
void update() {
    std::lock_guard lock(mutex);

    bool top_jammed = motorJammed(top_motor);

    if (antijam_active && top_jammed) {
        // antijam is active and bottom is jammed, start doing something
        if (scoring_antijam) {
            // scoring antijam is active
            bool initial_timeout_done =
              timeoutDone(initial_timeout, score_start_time);

            if (initial_timeout_done) {
                // move at full speed in opposite direction of desired pct
                hardware_move_pct(units::sgn(pct) * -1.0_volt);

                // scoring antijam action
                pros::delay(to_msec(antijam_timeout));

                // afterwards move as normal
                hardware_move_pct(pct);

                // give time to settle
                pros::delay(to_msec(settle_time));

                // afterwards goes through update again, if still jammed then
                // antijams again
            }
        }
    } else {
        // no antijam active, move like normal
        hardware_move_pct(pct);
    }
}

} // namespace top
// some more helper functions to make declaring states easier
//
// sets pct for both intake motors
void set_pct(auto pct) {
    bottom::set_pct(pct);
    top::set_pct(pct);
}

// sets pct for both intake motors
void set_pct(auto bottom, auto top) {
    bottom::set_pct(bottom);
    top::set_pct(top);
}

// sets antijam for both
void set_antijam(bool active) {
    top::set_antijam(true);
    bottom::set_antijam(true);
}

// sets antijam for both
void set_antijam(bool bottom_active, bool top_active) {
    top::set_antijam(true);
    bottom::set_antijam(true);
}

//
// common intake states are defined here
//
// only pauses motors, does not change piston states
void motors_disabled() {
    set_pct(0.0);
}

void intake() {
    set_pct(1.0);
    // does not set alignment
    pistons::scoring_blocked();
    pistons::intake_down();
}

void outtake() {
    set_pct(-1.0);
    // does not set alignment
    pistons::scoring_blocked();
    pistons::intake_down();
    top::set_scoring(false);
}

void score_long(float bottom_speed, float top_speed) {
    // default is full speed
    set_pct(bottom_speed, top_speed);

    pistons::score_top_aligned();
    pistons::intake_down();
    top::set_scoring(true);
}

void score_middle(float bottom_speed, float top_speed) {
    // default is fast on bottom, scores slower on top motor for middle goal
    set_pct(bottom_speed, top_speed);

    pistons::score_middle_aligned();
    pistons::intake_down();
    top::set_scoring(true);
}

void score_middle_slow() {
    // score middle even slower
    score_middle(1.0, 0.25);
}

void score_bottom(float bottom_speed, float top_speed) {
    // defaults to fast on top, slower on bottom
    set_pct(bottom_speed, top_speed);

    // only updates bottom piston, no need to update others
    pistons::intake_up();
    top::set_scoring(false);
}

void score_bottom_slow() {
    score_middle(-0.3, -1.0);
}

namespace driver {
void update() {
    if (!is_driver) return;

    // update states based on driver input
    bool driver_intake = controller.get_digital(controls::L1);

    bool score_bottom_height = controller.get_digital(controls::L2);
    bool score_middle_height = controller.get_digital(controls::R2);
    bool scoreLong = controller.get_digital(controls::R1);

    bool kill_color_sort = controller.get_digital_new_press(controls::LEFT);

    bool unjam = controller.get_digital(controls::X);

    // one time kill switch
    if (color_sort_driver == true && kill_color_sort) {
        color_sort_driver = false;
    }

    else if (unjam)
        outtake();

    else if (driver_intake)
        intake();

    else if (score_middle_height) {
        score_middle();
    }

    else if (score_bottom_height) {
        score_bottom();
    }

    else if (scoreLong) {
        score_long();
    } else {
        motors_disabled();
    }
}
} // namespace driver

// initializes pistons, top, and bottom systems
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
    setup_color_sensor(bottom_intake_color_sensor);

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

    pros::Task top_motor_task(
      [] {
          while (true) {
              top::update();
              pros::delay(10);
          }
      },
      "top motor task");

    pros::Task driver_update_task(
      [] {
          while (true) {
              driver::update();
              pros::delay(10);
          }
      },
      "intake driver task");

    tasks_active = true;
}

} // namespace new_intake
