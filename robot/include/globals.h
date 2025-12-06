#pragma once

#include "apis.h"

namespace controls {
const auto L1 = pros::E_CONTROLLER_DIGITAL_L1;
const auto R1 = pros::E_CONTROLLER_DIGITAL_R1;

const auto L2 = pros::E_CONTROLLER_DIGITAL_L2;
const auto R2 = pros::E_CONTROLLER_DIGITAL_R2;

const auto X = pros::E_CONTROLLER_DIGITAL_X;
const auto Y = pros::E_CONTROLLER_DIGITAL_Y;
const auto A = pros::E_CONTROLLER_DIGITAL_A;
const auto B = pros::E_CONTROLLER_DIGITAL_B;

const auto UP = pros::E_CONTROLLER_DIGITAL_UP;
const auto DOWN = pros::E_CONTROLLER_DIGITAL_DOWN;
const auto LEFT = pros::E_CONTROLLER_DIGITAL_LEFT;
const auto RIGHT = pros::E_CONTROLLER_DIGITAL_RIGHT;

const auto LEFT_SHIFT = pros::E_CONTROLLER_DIGITAL_RIGHT;
const auto RIGHT_SHIFT = pros::E_CONTROLLER_DIGITAL_Y;
} // namespace controls

using namespace blazing;

#include "globals/blazing_globals.h"
#include "globals/config.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
