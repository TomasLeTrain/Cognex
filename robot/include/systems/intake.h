#pragma once

#include "auton_globals.h"
#include <optional>

namespace intake {
enum intake_state_t {
    intake_disabled,
    intake_disabled_open_middle,
    outtake_very_slow,
    outtake_bottom_balls_open_middle,

    score_bottom_slow,

    intake,
    intake_bottom_balls,
    intake_bottom_top_backwards,
    intake_top_balls,
    outtake_bottom_balls,
    outtake,
    outtake_open_middle,

    unjam,

    slow_scoring_middle,
    slow_scoring_bottom,

    scoring_middle,
    scoring_middle_top_balls,
    scoring_middle_first_ball,
    scoring_middle_top_balls_skills,
    scoring_middle_top_balls_skills_fast,
    scoring_middle_top_balls_skills_slow,

    scoring_middle_bottom_balls,
    scoring_middle_bottom_balls_slow,

    score_bottom_bottom_balls,
    score_bottom_bottom_balls_slow,

    scoring_bottom_driver,
    scoring_bottom,
    scoring_long,

    // scores on long but only scores top balls
    scoring_long_top_balls,
    scoring_long_top_balls_outake_bottom,

    scoring_middle_bottom_balls_awp,
};

enum intake_piston_state_t {
    // blocks balls from going through
    blocking,
    // allows balls to go through it
    passthrough
};

enum bottom_intake_piston_state_t {
    up,
    down
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
void setDriverColorSortEnabled(bool enabled);

// returns color detected in the middle of the intake
std::optional<alliance_t> getMiddleDetectedColor();
// returns color detected in the bottom of the intake
std::optional<alliance_t> getBottomDetectedColor();

// waits up to a max of timeout time waiting for color (any color if nullopt) to
// appear in middle of intake, and if detected then throws it out.
//
// Useful as very temporary color sort or manual purging of a ball.
void throwOutDetectedBall(std::optional<alliance_t> color, Time timeout);

// util that waits a max of timeout time to find ball of specified color (any
// color of nullopt) in bottom intake.
//
// returns true if ball was detected, and false if timeout was triggered.
bool waitUntilBottomColor(std::optional<alliance_t> color, Time timeout);

// util that waits a max of timeout time to find ball of specified color (any
// color of nullopt) in middle intake.
//
// returns true if ball was detected, and false if timeout was triggered.
bool waitUntilMiddleColor(std::optional<alliance_t> color, Time timeout);

void init(bool gdriver);

// helper tasks to do common stuff
void in();
void out();
void score_long();
void score_middle();
void score_bottom();

void setSkillsMiddleScoring(bool enabled);
void setBottomUnjamDisabled(bool disabled);

} // namespace intake
