#include "apis.h"
//
#include "globals.h"
#include "systems/odom_retract.h"
#include "systems/piston.h"

namespace odom_retract {
bool is_driver = false;
bool tasks_active = false;

piston_state_t odom_retract_state = inactive;

/**
 * @brief updates intake state, with optional speed parameter
 *
 * @param new_wings_state new intake state
 */
void set(piston_state_t new_wings_state) {
  odom_retract_state = new_wings_state;
}

// code that should run during driver
void driverUpdate() {
  // update states based on driver input
  bool toggle_piston_retract =
      controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
  if (toggle_piston_retract) {
    if (odom_retract_state == active)
      set(inactive);
    else
      set(active);
  }
}

// code that should run during autonomous - should be based on extra state
// specific to autonomous
void autoUpdate() {}

// updates the physical motor to match the current state of the subsystem
// runs regardless of driver mode
void hardwareUpdate() {
  // intake update
  if (odom_retract_state == active) {
    odom_retract_piston.set_value(true);
  } else {
    odom_retract_piston.set_value(false);
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
  if (tasks_active)
    return;

  // run any code here that should only occur once

  pros::Task odom_retract_task([] {
    while (true) {
      update();
      pros::delay(10);
    }
  });

  tasks_active = true;
}
}; // namespace odom_retract
