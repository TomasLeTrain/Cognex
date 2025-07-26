#include "systems/intake.h"
#include "globals.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"

namespace intake {
// any needed variables can be specified here
// NOTE: it is recommended these variables are not changed directly by
// autos/other subsystems you should ideally provide functions to interact with
// these variables
bool is_driver = false;
bool tasks_active = false;

intake_state_t intake_state = intake_disabled;
score_state_t score_state = score_disabled;
bin_state_t bin_state = bin_disabled;

// absolute speed of the motor. Should always be positive
int intake_speed = 127;
int score_speed = 127;
int bin_speed = 127;

// used by intake specific functions - probably bad idea to expose (use set() in
// autos instead)
void intakeToggle() {
    if (intake_state == intake_disabled) {
        intake_state = intake;
    } else if (intake_state == intake || intake_state == outtake) {
        intake_state = intake_disabled;
    }
}

void intakeToggleDirection() {
    if (intake_state == intake) {
        intake_state = outtake;
    } else if (intake_state == outtake) {
        intake_state = intake;
    } else if (intake_state == intake_disabled) {
        // intake not on anyway
    }
}

// code that should run during driver
void driverUpdate() {
    // update states based on driver input
    int toggle_intake =
      controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
    int toggle_intake_direction =
      controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);

    if (toggle_intake) {
        intakeToggle();
    }
    if (toggle_intake_direction) {
        intakeToggleDirection();
    }
    // TODO: add scoring and bin logic
}

// code that should run during autonomous - should be based on extra state specific to autonomous
void autoUpdate() {

}

// updates the physical motor to match the current state of the subsystem
// runs regardless of driver mode
void motorUpdate() {
    // intake update
    if (intake_state == intake_disabled) {
        intake_motor.move(0);
    } else if (intake_state == intake) {
        intake_motor.move(intake_speed);
    } else if (intake_state == outtake) {
        intake_motor.move(-intake_speed);
    }

    // scoring update - done after since it likely overrides intake in most cases
    if (score_state == score_disabled) {
        score_motor.move(0);
    } else if (score_state == scoring_top) {
        // could change its speed here if neccesary
        score_motor.move(score_speed);
    } else if (score_state == scoring_long) {
        score_motor.move(score_speed);
    } else if(score_state == scoring_bottom){
        // NOTE: special case where scoring affects intake!
        intake_motor.move(-score_speed);
    }

    // bin update
    if (bin_state == bin_disabled) {
        bin_motor.move(0);
    } else if (bin_state == fill) {
        bin_motor.move(score_speed);
    } else if (bin_state == take_out) {
        bin_motor.move(-score_speed);
    }
}

// updates the state of the subsystem
void update() {
    // any code that needs to run regardless of driver mode can also run here
    if (is_driver) {
        driverUpdate();
    } else {
        autoUpdate();
    }
    motorUpdate();
}

void init(bool gdriver) {
    is_driver = gdriver;

    // don't make another task
    if (tasks_active) return;

    // run any code here that should only occur once

    pros::Task main_intake_task([] {
        while (true) {
            update();
            pros::delay(10);
        }
    });

    tasks_active = true;
}

/*
 * setters and getters - meant to be used by autons/subsystems outside this file
 * should also be specified in the intake.h file */

/**
 * @brief updates scoring state, with optional speed parameter
 *
 * @param new_score_state new scoring state
 * @param new_score_speed speed at which to run the scoring. Defaults to max
 * speed
 */
void setScore(score_state_t new_score_state, int new_score_speed) {
    score_state = new_score_state;
    score_speed = abs(new_score_speed);

    // scoring on the bottom goal is a special case since it requires the use of
    // the intake motor. to stop any possible conflicts we change it here as well
    if (score_state == scoring_bottom) {
        intake_state = intake_disabled;
    }
}

/**
 * @brief updates intake state, with optional speed parameter
 *
 * @param new_intake_state new intake state
 * @param new_intake_speed speed at which to run the intake. Defaults to max
 * speed
 */
void setIntake(intake_state_t new_intake_state, int new_intake_speed) {
    intake_state = new_intake_state;
    intake_speed = abs(new_intake_speed);
    // scoring on the bottom goal is a special case since it requires the use of
    // the intake motor. to stop any possible conflicts we change it here as well
    if (intake_state != intake_disabled) {
        score_state = score_disabled;
    }
}

/**
 * @brief updates bin state, with optional speed parameter
 *
 * @param new_bin_state new bin state
 * @param new_bin_speed speed at which to run the bin. Defaults to max speed
 */
void setBin(bin_state_t new_bin_state, int new_bin_speed) {
    bin_state = new_bin_state;
    bin_speed = abs(new_bin_speed);
}

}; // namespace intake
