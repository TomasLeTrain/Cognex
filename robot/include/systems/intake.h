#pragma once

namespace intake {
enum score_state_t {
    score_disabled = 0,
    scoring_top = 1,
    scoring_bottom = 2,
    scoring_long = 3
};

enum intake_state_t {
    intake_disabled = 0,
    intake = 1,
    outtake = -1
};

enum bin_state_t {
    bin_disabled = 0,
    fill = 1,
    take_out = -1
};

/**
 * @brief updates scoring state, with optional speed parameter
 *
 * @param new_score_state new scoring state
 * @param new_score_speed speed at which to run the scoring. Defaults to max
 * speed
 */
void setScore(score_state_t new_score_state, int new_score_speed);

/**
 * @brief updates intake state, with optional speed parameter
 *
 * @param new_intake_state new intake state
 * @param new_intake_speed speed at which to run the intake. Defaults to max
 * speed
 */
void setIntake(intake_state_t new_intake_state, int new_intake_speed);
/**
 * @brief updates bin state, with optional speed parameter
 *
 * @param new_bin_state new bin state
 * @param new_bin_speed speed at which to run the bin. Defaults to max speed
 */
void setBin(bin_state_t new_bin_state, int new_bin_speed);

void init(bool gdriver);
} // namespace intake
