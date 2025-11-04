#include "apis.h"
//
#include "globals.h"
#include "systems/matchloader.h"

namespace matchloader {
bool is_driver = false;
bool tasks_active = false;

matchloader_state_t matchloader_state = inactive;

/**
 * @brief updates intake state, with optional speed parameter
 *
 * @param new_matchloader_state new intake state
 */
void set(bool new_matchloader_state) {
    matchloader_state = matchloader_state_t(new_matchloader_state);
}

void set(matchloader_state_t new_matchloader_state) {
    matchloader_state = new_matchloader_state;
}

// code that should run during driver
void driverUpdate() {
    // update states based on driver input
    bool toggleMatchloader =
      controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);

    set(toggleMatchloader);
}

// code that should run during autonomous - should be based on extra state
// specific to autonomous
void autoUpdate() {}

// updates the physical motor to match the current state of the subsystem
// runs regardless of driver mode
void hardwareUpdate() {
    // intake update
    if (matchloader_state) {
        matchloader_piston.set_value(true);
    } else {
        matchloader_piston.set_value(false);
    }
}

// updates the state of the subsystem
void update() {
    if (is_driver) {
        driverUpdate();
    } else {
        autoUpdate();
    }
    hardwareUpdate();
}

void init(bool gdriver) {
    is_driver = gdriver;

    // don't make another task
    if (tasks_active) return;

    // run any code here that should only occur once

    pros::Task main_matchloader_task([] {
        while (true) {
            update();
            pros::delay(10);
        }
    });

    tasks_active = true;
}
}; // namespace matchloader
