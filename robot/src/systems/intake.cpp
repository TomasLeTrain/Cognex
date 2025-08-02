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
    { slow_scoring_bottom, -60  },
    { scoring_bottom,      -127 },

    { slow_scoring_middle, 70   },
    { scoring_middle,      127  },

    { scoring_long,        110  },

    { intake,              127  },
    { intake_slow_bottom,  60   },

    { priming,             0    },
    { unjam,               -127 },
};

std::map<intake_state_t, int> bin_motor_speeds = {
    { slow_scoring_bottom, -50  },
    { scoring_bottom,      -70  },

    { slow_scoring_middle, -50  },
    { scoring_middle,      -70  },

    { scoring_long,        -127 },

    { intake,              0    },
    { intake_slow_bottom,  0    },

    { priming,             0    },
    { unjam,               127  },
};

std::map<intake_state_t, int> score_motor_speeds = {
    { scoring_bottom,      0    },
    { slow_scoring_bottom, 0    },

    { slow_scoring_middle, -20  },
    { scoring_middle,      -20  },

    { scoring_long,        127  },

    { intake,              127  },
    { intake_slow_bottom,  1227 },

    { priming,             0    },
    { unjam,               127  },
};

// speeds of the motors - can be positive or negative
int intake_speed;
int score_speed;
int bin_speed;

bool tmp_activated = false;
bool last_tmp_activated = true;

std::optional<alliance_t> last_middle_detected_color;

std::optional<alliance_t> middle_detected_color;
std::optional<alliance_t> bottom_detected_color;

pros::Mutex intake_mutex;

bool colorSortEnabled = true;
bool driverColorSortEnabled = true;

bool prime_active = false;

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
    if (new_intake_state == priming) {
        prime_active = true;
    } else {
        prime_active = false;
    }
}

void setColorSortEnabled(bool enabled) {
    colorSortEnabled = enabled;
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
    bool scoreLong = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);

    bool slowScoring = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    bool toggleColorSort =
      controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT);

    bool unjam = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

    bool primeMacro =
      controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
      controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT);

    if (toggleColorSort) driverColorSortEnabled = !driverColorSortEnabled;

    if (primeMacro) {
        set(intake_state_t::priming);
        return;
    }

    if (unjam) {
        set(intake_state_t::unjam);
    } else if (intakeToBackpack) {
        set(intake_state_t::intake);
    } else if (scoreMiddleHeight) {
        // change the intake state based on the speed
        if (slowScoring) {
            set(intake_state_t::slow_scoring_middle);
        } else {
            set(intake_state_t::scoring_middle);
        }
    } else if (scoreBottomHeight) {
        if (slowScoring) {
            set(intake_state_t::slow_scoring_bottom);
        } else {
            set(intake_state_t::scoring_bottom);
        }
    } else if (scoreLong) {
        // not active and last was not active either
        if (last_tmp_activated) {
            tmp_activated = true;
            last_tmp_activated = false;
        }

        set(intake_state_t::scoring_long);
        // only disable if prime was not active
    } else if (!prime_active) {
        set(intake_state_t::intake_disabled);
        last_tmp_activated = true;
    }
}

std::optional<alliance_t> colorDetected(pros::Optical* sensor) {
    // detect color from color sensor
    std::optional<alliance_t> result = std::nullopt;

    // just say no balls are being detected
    if (!sensor->is_installed()) return result;

    double color_sensor_hue = sensor->get_hue();

    // intake senses something
    if (sensor->get_proximity() > 70) {
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

bool motorJammed(pros::Motor* motor) {
    return (std::abs(motor->get_voltage()) > 10 && motor->get_torque() > 0.1 &&
            std::fabs(motor->get_actual_velocity()) < 1);
}

bool motorSlowed(pros::Motor* motor) {
    return (std::abs(motor->get_voltage()) > 10 && motor->get_torque() > 0.1 &&
            std::fabs(motor->get_actual_velocity()) < 100);
}

// should be run in a task
void antiJam() {
    while (true) {
        // wait for stuff to be available
        intake_mutex.take();

        bool score_motor_slowed = motorSlowed(&score_motor);

        if (score_motor_slowed && intake_state == scoring_long &&
            !tmp_activated) {
            // ) {
            intake_motor.move(0);
            // make sure bin would not continue running which would jam
            bin_motor.move(0);

            pros::delay(200);
            intake_motor.move(intake_speed);
            score_motor.move(score_speed);
            bin_motor.move(bin_speed);
        }
        if ((motorJammed(&intake_motor) || motorJammed(&score_motor)) &&
            intake_state == intake) {
            intake_motor.move(-127);
            score_motor.move(-127);
            intake_recycle_piston.set_value(true);
            // make sure bin would not continue running which would jam
            bin_motor.move(0);

            pros::delay(200);
            intake_recycle_piston.set_value(false);
            intake_motor.move(intake_speed);
            score_motor.move(score_speed);
            bin_motor.move(bin_speed);
        }

        if (motorJammed(&intake_motor) && intake_state == scoring_middle) {
            // try and unjam if unaligned
            intake_motor.move(-127);
            bin_motor.move(0);
            // make sure bin would not continue running which would jam

            pros::delay(300);
            intake_motor.move(intake_speed);
            bin_motor.move(bin_speed);
        }

        if (motorJammed(&intake_motor) && intake_state == scoring_bottom) {
            // try and unjam if unaligned
            intake_motor.move(127);
            bin_motor.move(60);
            // make sure bin would not continue running which would jam

            pros::delay(200);
            intake_motor.move(intake_speed);
            bin_motor.move(bin_speed);
        }

        if (motorJammed(&bin_motor)) {
            if (intake_state == scoring_middle ||
                intake_state == slow_scoring_middle ||
                intake_state == scoring_bottom ||
                intake_state == slow_scoring_bottom) {
                // make it not stuck
                bin_motor.move(127);
                pros::delay(200);
                bin_motor.move(bin_speed);
            } else {
                // make it not stuck
                bin_motor.move(-127);
                pros::delay(200);
                bin_motor.move(bin_speed);
            }
        }

        // not slowed, can disable the temporary
        if (!score_motor_slowed && tmp_activated) {
            tmp_activated = false;
        }

        intake_mutex.give();

        pros::delay(10);
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
    while (true) {
        // wait for stuff to be available
        // does not change intake, should not depend on mutex

        if ((!is_driver && !colorSortEnabled) ||
            (is_driver && !driverColorSortEnabled) ||
            // we don't know our alliance so we cannot color sort
            auto_alliance == alliance_t::unset) {
            pros::delay(10);
            return;
        }

        // a ball is being measured
        bool middle_wrong_color_detected = false;

        if (middle_detected_color.has_value()) {
            middle_wrong_color_detected =
              middle_detected_color.value() != auto_alliance;
        }
        // the detected ball is not the our alliance
        // we selected an autonomous alliance

        // bool bottom_wrong_color_detected =
        //   // a ball is being measured
        //   bottom_detected_color != std::nullopt &&
        //   // the detected ball is not the our alliance
        //   bottom_detected_color.value() != auto_alliance &&
        //   // we selected an autonomous alliance
        //   auto_alliance != alliance_t::unset;

        // we have the wrong color, prcoess based on current state
        if (middle_wrong_color_detected) {
            // try to outake through the middle of the intake
            if (intake_state == intake) {
                // need to take mutex
                intake_mutex.take();

                score_motor.move(60);
                intake_motor.move(60);
                intake_recycle_piston.set_value(false);
                pros::delay(400);
                intake_recycle_piston.set_value(true);

                // score_motor.move(score_speed);
                // intake_motor.move(intake_speed);

                intake_mutex.give();
            } else if (intake_state == scoring_long) {
                // recycle ball into bin
                intake_mutex.take();

                // give a bit of delay for the ball to travel up
                // pros::delay(100);

                // wait for opposite color to appear at the top of the intake
                // set to recycle
                intake_recycle_piston.set_value(true);

                // wait for ball to go into bin
                pros::delay(150);
                intake_recycle_piston.set_value(false);

                intake_mutex.give();
            } else if (intake_state == scoring_middle ||
                       intake_state == slow_scoring_middle) {
                // recycle ball into bin
                intake_mutex.take();

                // wait for opposite color to appear at the top of the intake
                // set to recycle
                score_motor.move(127);
                bin_motor.move(0);
                intake_motor.move(0);

                // wait for ball to go into bin
                pros::delay(200);
                score_motor.move(score_speed);
                bin_motor.move(bin_speed);
                intake_motor.move(intake_speed);

                intake_mutex.give();
            }
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

    // only update motors if they are not being used elsewhere - waits for 2
    // millisecends to be able to use
    if (intake_mutex.take(2)) {
        if (intake_state == intake || intake_state == scoring_middle ||
            intake_state == slow_scoring_middle ||
            intake_state == intake_slow_bottom ||
            intake_state == intake_disabled)
            intake_recycle_piston.set_value(true);
        else
            intake_recycle_piston.set_value(false);

        if (intake_state == scoring_bottom ||
            intake_state == slow_scoring_bottom)
            intake_raise_piston.set_value(true);
        else
            intake_raise_piston.set_value(false);

        // priming is a special mode, don't use normal speeds
        if (intake_state == priming && prime_active) {
            // intake_motor.move(intake_motor_speeds[scoring_long]);
            // bin_motor.move(bin_motor_speeds[scoring_long]);
            // score_motor.move(score_motor_speeds[scoring_long]);

            intake_motor.move(80);
            bin_motor.move(-100);
            score_motor.move(127);

            // if (middle_detected_color != last_middle_detected_color) {
            if (middle_detected_color.has_value()) {
                // int thingy1 = -1;
                // int thingy2 = -1;
                // if(middle_detected_color.has_value()) thingy1 =
                // (int)middle_detected_color.value();
                // if(last_middle_detected_color.has_value()) thingy2 =
                // (int)last_middle_detected_color.value(); printf("new
                // balls(new,last)! %d %d\n",thingy1,thingy2); we definetly have
                // a many balls as we want

                intake_motor.move(0);
                bin_motor.move(0);
                score_motor.move(0);
                prime_active = false;
            }
        } else {
            intake_motor.move(intake_speed);
            bin_motor.move(bin_speed);
            score_motor.move(score_speed);
        }

        intake_mutex.give();
    }
}

// updates the state of the subsystem
void update() {
    // any code that needs to run regardless of driver mode can also run here

    // update current detected color
    last_middle_detected_color = middle_detected_color;

    middle_detected_color = colorDetected(&middle_intake_color_sensor);
    bottom_detected_color = colorDetected(&bottom_intake_color_sensor);

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
    if (middle_intake_color_sensor.is_installed()) {
        middle_intake_color_sensor.disable_gesture();
        middle_intake_color_sensor.set_integration_time(10);
        middle_intake_color_sensor.set_led_pwm(100);
    }

    if (bottom_intake_color_sensor.is_installed()) {
        bottom_intake_color_sensor.disable_gesture();
        bottom_intake_color_sensor.set_integration_time(10);
        bottom_intake_color_sensor.set_led_pwm(100);
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
