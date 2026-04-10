#pragma once

#include "apis.h"
//

#include "auton_globals.h"
#include "pros/motors.hpp"
#include "units/units.hpp"

namespace intake {
void setAutonColorSort(bool enabled);
void setDriverColorSort(bool enabled);

bool motorJammed(pros::Motor& motor);
bool motorSlowed(pros::Motor& motor);

namespace colors {
std::optional<alliance_t> colorDetected(pros::Optical& sensor);

std::optional<alliance_t> getLowerColor();
std::optional<alliance_t> getUpperColor();
void update();
}; // namespace colors

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
void gate_blocked();

// sets the top scoring to be passthrough
void gate_scoring();

void align_top();
void align_middle();
void intake_up();
void intake_down();

} // namespace pistons

namespace bottom {
void set_pct(Voltage new_pct);
void set_pct(float new_pct);
// void set_rpm(AngularVelocity new_rpm);
// void set_rpm_pct(float new_vel_pct);
void set_antijam(bool active);
void set_outtake_antijam(bool active);

} // namespace bottom

namespace lever {
enum DiscreteLeverState {
    following_profile,
    position_reset,
    going_down,
    down,
    going_up,
    up
};

struct LeverVelocityProfile {
    std::function<float(float)> m_f;

    LeverVelocityProfile(std::function<float(float)> f);

    // returns the desired target velocity at angle theta in range [0,1]
    float velocity(float theta);

    // determines when the profile is finished with a motion
    // if finished, returns a discrete lever state to go back to
    // TODO: or maybe a target theta?
    std::optional<DiscreteLeverState> finished(float theta);
};

void setActionVoltage(Voltage actionVoltage);

void setTarget(
  std::variant<Voltage, DiscreteLeverState, float, LeverVelocityProfile>
    target);
std::variant<Voltage, DiscreteLeverState, float, LeverVelocityProfile>
getTarget();
float getLeverPosition();

void hardware_update(Voltage voltage);

} // namespace lever

//
// common intake states are defined here
//

// sets the target of the lever such that eventually it reaches the down state
// and stays there
void continuous_lever_down();

// sets the target of the lever such that eventually it reaches the down state
// and stays there
void continuous_lever_up(Voltage actionVoltage = 1_volt);

//
// only pauses motors, does not change piston states
void motors_disabled();

void in();
void out();

void score_long(Voltage actionVoltage = 1_volt);

void score_middle(Voltage actionVoltage = 0.5_volt);

void score_bottom();

namespace driver {
// void update();
} // namespace driver

// initializes pistons, lever, and bottom systems
void init(bool driver);

} // namespace intake
