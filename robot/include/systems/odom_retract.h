#pragma once

#include "systems/piston.h"

namespace odom_retract {
void set(piston_state_t new_wings_state);

void init(bool gdriver);
} // namespace wings
