#include "systems/intake.h"
#include "globals.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
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

std::map<intake_state_t, int> intake_motor_speeds = {
    // only different one
    { slow_scoring_bottom, -40 },
    { scoring_bottom,      -80 },

    { slow_scoring_middle, 127 },
    { scoring_middle,      127 },

    { scoring_long,        127 },

    { intake,              127 },
};

std::map<intake_state_t, int> bin_motor_speeds = {
    { slow_scoring_bottom, -50 },
    { scoring_bottom,      -90 },

    { slow_scoring_middle, -50 },
    { scoring_middle,      -70 },

    { scoring_long,        -100 },

    { intake,              0   },
};

std::map<intake_state_t, int> score_motor_speeds = {
    { scoring_bottom,      0   },
    { slow_scoring_bottom, 0   },

    { slow_scoring_middle, -10 },
    { scoring_middle,      -10 },

    { scoring_long,        127 },
    { intake,              127 },
};

// speeds of the motors - can be positive or negative
int intake_speed;
int score_speed;
int bin_speed;

std::optional<alliance_t> current_detected_color;

bool recycling_enabled = false;

pros::Mutex intake_mutex;

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
}

// code that should run during driver
void driverUpdate() {
    // update states based on driver input
    bool intakeToBackpack =
      controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    bool scoreBottomHeight =
        controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    bool scoreMiddleHeight =
      controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    bool scoreLong =
      controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

    bool slowScoring =
      controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    if (intakeToBackpack) {
        set(intake_state_t::intake);
    }else if (scoreMiddleHeight) {
        // change the intake state based on the speed
        if (slowScoring) {
            set(intake_state_t::slow_scoring_middle);
        } else {
            set(intake_state_t::scoring_middle);
        }
    }else if (scoreBottomHeight) {
        if (slowScoring) {
            set(intake_state_t::slow_scoring_bottom);
        } else {
            set(intake_state_t::scoring_bottom);
        }
    }else if (scoreLong) {
        set(intake_state_t::scoring_long);
    }else{
        set(intake_state_t::intake_disabled);
    }
}

std::optional<alliance_t> colorDetected() {
    // detect color from color sensor
    current_detected_color = std::nullopt;

    // just say no balls are being detected
    if (!intake_color_sensor.is_installed()) return current_detected_color;

    int color_sensor_hue = intake_color_sensor.get_hue();

    // intake senses something
    if (intake_color_sensor.get_proximity() > 240) {
        if (color_sensor_hue > 340 || color_sensor_hue < 20)
            current_detected_color = alliance_t::red;
        else if (color_sensor_hue > 160 && color_sensor_hue < 270)
            current_detected_color = alliance_t::blue;
    }

    return current_detected_color;
}

// code that should run during autonomous - should be based on extra state
// specific to autonomous
void autoUpdate() {}

bool motorJammed(pros::Motor* motor) {
    return (std::abs(motor->get_voltage()) > 10 && motor->get_torque() > 0.1 &&
            std::fabs(motor->get_actual_velocity()) < 1);
}

// should be run in a task
void antiJam() {
    while (true) {
        // wait for stuff to be available
        intake_mutex.take();

        if (motorJammed(&intake_motor) || motorJammed(&score_motor)) {
            intake_motor.move(-intake_speed);
            score_motor.move(-score_speed);
            pros::delay(200);
            intake_motor.move(intake_speed);
            score_motor.move(score_speed);
        }

        if (motorJammed(&bin_motor)) {
            // make it not stuck
            bin_motor.move(-bin_speed);
            pros::delay(200);
            bin_motor.move(bin_speed);
        }

        intake_mutex.give();

        pros::delay(10);
    }
}

// should be run in a task
void colorSort() {
    while (true) {
        // wait for stuff to be available
        // does not change intake, should not depend on mutex

        bool changed_recycling = false;

        if (current_detected_color != std::nullopt) {
            if (current_detected_color.value() != auto_alliance &&
                auto_alliance != alliance_t::unset) {
                // we have the wrong color, prcoess based on current state
                if (intake_state == intake || intake_state == scoring_middle) {
                    // need to take mutex
                    intake_mutex.take();

                    // std::cout << "color sorting 1" << std::endl;

                    score_motor.move(-127);
                    intake_motor.move(0);
                    pros::delay(200);
                    score_motor.move(score_speed);
                    intake_motor.move(intake_speed);

                    intake_mutex.give();
                }else if (intake_state == scoring_long){
                    // std::cout << "color sorting 2" << std::endl;
                    intake_mutex.take();

                    // no need to get intake
                    recycling_enabled = true;
                    changed_recycling = true;

                    pros::delay(100);
                    intake_recycle_piston.set_value(true);
                    intake_mutex.give();
                }
            }
        }
        if(!changed_recycling && recycling_enabled){
            // should change recycling state back to normal
            intake_mutex.take();
            pros::delay(100);
            intake_recycle_piston.set_value(false);
            intake_mutex.give();
        }

        pros::delay(10);
    }
}

// updates the physical motor to match the current state of the subsystem
// runs regardless of driver mode
void hardwareUpdate() {
    // intake update

    int intake_speed = intake_motor_speeds[intake_state];
    int bin_speed = bin_motor_speeds[intake_state];
    int score_speed = score_motor_speeds[intake_state];

    // only update motors if they are not being used elsewhere
    if (intake_mutex.take(2)) {
        if(intake_state == intake) intake_recycle_piston.set_value(true);
        else intake_recycle_piston.set_value(false);

        intake_motor.move(intake_speed);
        bin_motor.move(bin_speed);
        score_motor.move(score_speed);

        intake_mutex.give();
    }
}

// updates the state of the subsystem
void update() {
    // any code that needs to run regardless of driver mode can also run here

    // update current detected color
    auto color = colorDetected();

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
    if(intake_color_sensor.is_installed()){
        intake_color_sensor.disable_gesture();
        intake_color_sensor.set_integration_time(30);
    }

    // don't make another task
    if (tasks_active) return;

    // run any code here that should only occur once


    pros::Task antijam_task([] {
        while (true) {
            antiJam();
            pros::delay(10);
        }
    });
    //
    // pros::Task colorsort_task([] {
    //     while (true) {
    //         colorSort();
    //         pros::delay(10);
    //     }
    // });

    pros::Task main_intake_task([] {
        while (true) {
            update();
            pros::delay(10);
        }
    });

    tasks_active = true;
}
}; // namespace intake
