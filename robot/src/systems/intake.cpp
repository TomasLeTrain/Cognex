#include "apis.h"
//
#include "globals.h"
#include "globals/device_globals.h"
#include "pros/rtos.hpp"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include <map>
#include <mutex>

namespace intake {
// any needed variables can be specified here
// NOTE: it is recommended these variables are not changed directly by
// autos/other subsystems you should ideally provide functions to interact with
// these variables
bool is_driver = false;
bool tasks_active = false;

intake_state_t intake_state = intake_disabled;

std::map<intake_state_t, int> bottom_motor_speeds = {
    // only different one
    { slow_scoring_bottom, -60  },
    { scoring_bottom,      -110 },

    { slow_scoring_middle, 127  },
    { scoring_middle,      127  },

    { scoring_long,        127  },

    { intake,              127  },
    { intake_slow_bottom,  127  },

    { priming,             0    },
    { unjam,               -127 },
};

std::map<intake_state_t, int> top_motor_speeds = {
    { slow_scoring_bottom, -40  },
    { scoring_bottom,      -80  },

    { slow_scoring_middle, 127  },
    { scoring_middle,      127  },

    { scoring_long,        127  },

    { intake,              127  },
    { intake_slow_bottom,  65   },

    { priming,             0    },
    { unjam,               -127 },
};

// speeds of the motors - can be positive or negative
int bottom_speed;
int top_speed;

bool tmp_activated = false;

// used to avoid the initial jam voltage disable
bool last_tmp_activated = true;

std::optional<alliance_t> last_middle_detected_color;

std::optional<alliance_t> middle_detected_color;
std::optional<alliance_t> bottom_detected_color;

pros::Mutex intake_mutex;

bool colorSortEnabled = true;
bool driverColorSortEnabled = true;

bool tmp_middle_active = false;

/*
 * setters and getters - meant to be used by autons/subsystems outside this file
 * should also be specified in the intake.h file */

/**
 * @brief updates intake state, with optional speed parameter
 *
 * @param new_intake_state new intake state
 * @param new_intake_speed speed at which to run the intake. Defaults to max
 * speed
 */
void set(intake_state_t new_intake_state) {
    intake_state = new_intake_state;

    // update prime state
    tmp_middle_active = new_intake_state == scoring_middle;
}

void setColorSortEnabled(bool enabled) {
    colorSortEnabled = enabled;
}

// code that should run during driver
void driverUpdate() {
    // update states based on driver input
    bool intakeToBackpack = controller.get_digital(controls::L1);

    bool scoreBottomHeight = controller.get_digital(controls::L2);
    bool scoreMiddleHeight = controller.get_digital(controls::R2);
    bool scoreLong = controller.get_digital(controls::R1);

    bool slowScoring = false;

    bool killColorSort = controller.get_digital_new_press(controls::LEFT);

    bool unjam = controller.get_digital(controls::X);

    // bool primeMacro = controller.get_digital(controls::RIGHT_SHIFT) &&
    //                   controller.get_digital_new_press(controls::LEFT_SHIFT);
    bool primeMacro = false;

    // one time kill switch
    if (driverColorSortEnabled == true && killColorSort) {
        driverColorSortEnabled = false;
    }

    if (primeMacro)
        set(intake_state_t::priming);

    else if (unjam)
        set(intake_state_t::unjam);

    else if (intakeToBackpack)
        set(intake_state_t::intake);

    else if (scoreMiddleHeight) {
        // change the intake state based on the speed
        if (slowScoring) {
            set(intake_state_t::slow_scoring_middle);
        } else {
            set(intake_state_t::scoring_middle);
        }
    }

    else if (scoreBottomHeight) {
        if (slowScoring) {
            set(intake_state_t::slow_scoring_bottom);
        } else {
            set(intake_state_t::scoring_bottom);
        }
    }

    else if (scoreLong) {
        // not active and last was not active either
        if (last_tmp_activated) {
            tmp_activated = true;
            last_tmp_activated = false;
        }

        set(intake_state_t::scoring_long);
        // only disable if prime was not active
    }

    else if (!tmp_middle_active) {
        set(intake_state_t::intake_disabled);
        last_tmp_activated = true;
    }
}

std::optional<alliance_t> colorDetected(pros::Optical& sensor) {
    // detect color from color sensor
    std::optional<alliance_t> result = std::nullopt;

    // just say no balls are being detected
    if (!sensor.is_installed()) return result;

    double color_sensor_hue = sensor.get_hue();

    // intake senses something
    if (sensor.get_proximity() > 80) {
        if (color_sensor_hue > 280 || color_sensor_hue < 100)
            result = alliance_t::red;
        else if (color_sensor_hue > 120 && color_sensor_hue < 280)
            result = alliance_t::blue;
    }

    return result;
}

// code that should run during autonomous - should be based on extra state
// specific to autonomous
void autoUpdate() {}

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

// should be run in a task
void antiJam() {
    // wait for stuff to be available
    std::lock_guard lock(intake_mutex);

    if (intake_state == scoring_long && motorJammed(bottom_motor)) {
        bottom_motor.move(-127);
        pros::delay(200);
    }
    if (intake_state == scoring_long && motorJammed(top_motor)) {
        top_motor.move(-127);
        pros::delay(100);
    }
}

void waitUntilBottomColor(alliance_t color, uint32_t timeout) {
    uint32_t start_time = pros::millis();
    // either timeout triggers
    while (pros::millis() - start_time < timeout &&
           // or we get the color we want
           color != bottom_detected_color) {
        pros::delay(10);
    }
}

void waitUntilMiddleColor(alliance_t color, uint32_t timeout) {
    uint32_t start_time = pros::millis();
    // either timeout triggers
    while (pros::millis() - start_time < timeout &&
           // or we get the color we want
           color != middle_detected_color) {
        pros::delay(10);
    }
}

// should be run in a task
void colorSort() {
    bool autoColorSortDisabled = !is_driver && !colorSortEnabled;
    bool driverColorSortDisabled = is_driver && !driverColorSortEnabled;

    // wait for stuff to be available
    // does not change intake, should not depend on mutex
    if (autoColorSortDisabled || driverColorSortDisabled ||
        // we don't know our alliance so we cannot color sort
        auto_alliance == alliance_t::unset) {
        pros::delay(10);
        return;
    }

    // a ball is being measured
    bool middle_wrong_color_detected = middle_detected_color
                                         .transform([](alliance_t detected) {
                                             return detected != auto_alliance;
                                         })
                                         .value_or(false);

    // bool bottom_wrong_color_detected =
    //   bottom_detected_color
    //     .transform([](alliance_t detected) {
    //         return detected != auto_alliance;
    //     })
    //     .value_or(false);

    // we have the wrong color, prcoess based on current state
    if (middle_wrong_color_detected) {
        // try to outake through the middle of the intake
        if (intake_state == intake && matchloader::get() == active) {
            // matchloading, should color sort through the back
            std::lock_guard lock(intake_mutex);

            // make sure no other balls are in the way since those would not get
            // color sorted out?
            bottom_motor.move(0);
            // move top most ball out through score side
            top_motor.move(127);
            top_intake_piston.set_value(true);

            // really long delay, could be really inconsistent
            pros::delay(800);
        } else if (intake_state == intake || intake_state == scoring_long) {
            // need to take mutex
            std::lock_guard lock(intake_mutex);

            // move balls towards center hole
            bottom_motor.move(127);

            // open up center
            middle_intake_piston.set_value(false);

            // move top motor backwards a bit to ensure it gets thrown out
            top_motor.move(-50);

            pros::delay(80);
        }
        // else if (intake_state == scoring_middle ||
        //                     intake_state == slow_scoring_middle) {
        //              // need to take mutex
        //              std::lock_guard lock(intake_mutex);
        //
        // 	// assuming the balls come from the bottom,
        //
        //              // move balls up
        //              bottom_motor.move(-60);
        //              // move top most ball out
        //              top_motor.move(127);
        //
        // 	// wait for ball to go outwards
        //              pros::delay(200);
        //          }
    }
}

// updates the physical motor to match the current state of the subsystem
// runs regardless of driver mode
void hardwareUpdate() {
    // intake update
    bottom_speed = bottom_motor_speeds[intake_state];
    top_speed = top_motor_speeds[intake_state];

    // only update motors if they are not being used elsewhere - waits for 2
    // millisecends to be able to use
    if (intake_mutex.take(2)) {
        top_intake_piston.set_value(intake_state == scoring_long);
        middle_intake_piston.set_value(intake_state != scoring_middle);

        // priming is a special mode, don't use normal speeds
        if (intake_state == scoring_middle && tmp_middle_active) {
            bottom_motor.move(-127);
            top_motor.move(-127);
            pros::delay(100);
            bottom_motor.move(70);
            top_motor.move(-127);
            pros::delay(400);
            tmp_middle_active = false;
        } else {
            bottom_motor.move(bottom_speed);
            top_motor.move(top_speed);
        }

        intake_mutex.give();
    }
}

// updates the state of the subsystem
void update() {
    // any code that needs to run regardless of driver mode can also run here

    // update current detected color
    last_middle_detected_color = middle_detected_color;

    middle_detected_color = colorDetected(middle_intake_color_sensor);
    bottom_detected_color = colorDetected(bottom_intake_color_sensor);

    if (is_driver) {
        driverUpdate();
    } else {
        autoUpdate();
    }
    hardwareUpdate();
}

void init(bool gdriver) {
    is_driver = gdriver;

    // set some hardware related options
    auto setup_color_sensor = [](pros::Optical& sensor) {
        if (sensor.is_installed()) {
            sensor.disable_gesture();
            sensor.set_integration_time(10);
            sensor.set_led_pwm(100);
        }
    };

    setup_color_sensor(middle_intake_color_sensor);
    setup_color_sensor(bottom_intake_color_sensor);

    // don't make another task
    if (tasks_active) return;

    // run any code here that should only occur once

    pros::Task antijam_task(
      [] {
          while (true) {
              antiJam();
              pros::delay(10);
          }
      },
      "antijam");

    pros::Task colorsort_task(
      [] {
          while (true) {
              colorSort();
              pros::delay(10);
          }
      },
      "colorsort");

    pros::Task main_intake_task(
      [] {
          while (true) {
              update();
              pros::delay(10);
          }
      },
      "intake task");

    tasks_active = true;
}

// helper tasks to do common stuff
void in() {
    set(intake);
}

void out() {
    set(outtake);
}

void score_long() {
    set(scoring_long);
}

void score_middle() {
    set(scoring_middle);
}

void score_bottom() {
    set(scoring_bottom);
}

}; // namespace intake
