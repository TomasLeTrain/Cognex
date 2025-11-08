#pragma once

#include "systems/piston.h"

namespace matchloader {
void set(bool new_matchloader_state);
piston_state_t get();
void set(piston_state_t new_matchloader_state);

void init(bool gdriver);
} // namespace intake
