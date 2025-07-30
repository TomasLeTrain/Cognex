#pragma once

namespace intake {
enum intake_state_t {
    intake_disabled,

    intake,
    outtake,

    slow_scoring_middle,
    slow_scoring_bottom,

    scoring_middle,
    scoring_bottom,
    scoring_long
};

/**
 * @brief updates intake state, with optional speed parameter
 *
 * @param new_intake_state new intake state
 * @param new_intake_speed speed at which to run the intake. Defaults to max
 * speed
 */
void set(intake_state_t new_intake_state);

// sets if color sort is enabled
void setColorSortEnabled(bool enabled);

void init(bool gdriver);
} // namespace intake
