#pragma once

namespace matchloader {
enum matchloader_state_t {
	inactive = 0,
	active = 1
};

void set(bool new_matchloader_state);
void set(matchloader_state_t new_matchloader_state);

void init(bool gdriver);
} // namespace intake
