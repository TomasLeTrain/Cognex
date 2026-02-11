#pragma once

#include "pros/motors.hpp"
#include "units/units.hpp"

namespace intake {
void setAutonColorSort(bool enabled);
void setDriverColorSort(bool enabled);

bool motorJammed(pros::Motor& motor);
bool motorSlowed(pros::Motor& motor);

namespace pistons {
enum top_state_t {
    blocking,
    passthrough
};

enum middle_state_t {
    aligned_top,
    aligned_middle
};

enum bottom_state_t {
    up,
    down
};

void set_top(top_state_t top_state);

void set_middle(middle_state_t middle_state);

void set_bottom(bottom_state_t bottom_state);

void update();

// sets the top scoring to be blocked
void scoring_blocked();

// sets the top scoring to be passthrough
void scoring_passthrough();

void align_top();

void align_middle();

void intake_up();

void intake_down();

// helper functions for various configurations
void blocked_top_aligned();

void blocked_middle_aligned();

void score_top_aligned();

void score_middle_aligned();

} // namespace pistons

namespace bottom {

void set_pct(Voltage new_pct);

void set_pct(float new_pct);

void set_antijam(bool active);

void update();

} // namespace bottom

namespace top {
void set_pct(Voltage new_pct);

void set_pct(float new_pct);

void set_antijam(bool active);

// update scoring status, used by antijam
// updating does not affect antijam active state
void set_scoring(bool is_scoring);

// update can be blocking if antijam or color sort are active
// while blocking it also locks the mutex
void update();

} // namespace top

void init(bool driver);

// sets pct for both intake motors
void set_pct(auto pct);

// sets pct for both intake motors
void set_pct(auto bottom, auto top);

// sets antijam for both
void set_antijam(bool active);

// sets antijam for both
void set_antijam(bool bottom_active, bool top_active);

//
//
//
// only pauses motors, does not change piston states
void motors_disabled();

void in();

// useful for intaking balls only for bottom goal
// defaults to bottom full speed, top disabled
void intake_middle_balls(float bottom_speed = 1.0, float top_speed = 0.0);

void out();

// default is full speed
void score_long(float bottom_speed = 1.0, float top_speed = 1.0);

// default is fast on bottom, scores slower on top motor for middle goal
void score_middle(float bottom_speed = 1.0, float top_speed = 0.4);

// even slower scoring middle
void score_middle_slow();

// defaults to fast on top, slower on bottom
void score_bottom(float bottom_speed = -0.5, float top_speed = -1.0);

void score_bottom_slow();

namespace driver {
void update();
} // namespace driver
}; // namespace intake
