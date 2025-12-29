#pragma once

#include "systems/piston.h"

namespace wings {
void set(piston_state_t new_wings_state);
void up();
void down();

void init(bool gdriver);
} // namespace wings
