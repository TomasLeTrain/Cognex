#pragma once

#include "systems/piston.h"

namespace odom_retract {
void set(piston_state_t new_wings_state);

void retractOdom();
void lowerOdom();

piston_state_t get();

void init(bool gdriver);
} // namespace odom_retract
