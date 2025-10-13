#pragma once

#include "apis.h"
//
#include <string>

/* auton related stuff - can be left alone */
enum class alliance_t {
    unset,
    red,
    blue
};

enum class field_side_t {
    unset,
    left,
    right
};

extern alliance_t auto_alliance;
extern field_side_t auto_side;

extern std::string selected_auton;
