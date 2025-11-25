#include "apis.h"
//
#include "globals.h"
#include "systems/piston.h"
#include "systems/wings.h"

namespace wings {
bool is_driver = false;
bool tasks_active = false;

piston_state_t wings_state = inactive;

/**
 * @brief updates intake state, with optional speed parameter
 *
 * @param new_wings_state new intake state
 */
void set(piston_state_t new_wings_state) {
    wings_state = new_wings_state;
}

// code that should run during driver
void driverUpdate() {
    // update states based on driver input
    bool toggleWings = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

    set(piston_state_t(toggleWings));
}

// code that should run during autonomous - should be based on extra state
// specific to autonomous
void autoUpdate() {}

// updates the physical motor to match the current state of the subsystem
// runs regardless of driver mode
void hardwareUpdate() {
    // intake update
    if (wings_state == active) {
        wings_piston.set_value(true);
    } else {
        wings_piston.set_value(false);
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

    pros::Task main_wings_task([] {
        while (true) {
            update();
            pros::delay(10);
        }
    },"wings task");

    tasks_active = true;
}
}; // namespace wings
