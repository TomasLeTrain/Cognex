#pragma once

#include "systems/piston.h"

namespace matchloader {
void set(bool new_matchloader_state);
void set(piston_state_t new_matchloader_state);

void init(bool gdriver);
} // namespace intake
