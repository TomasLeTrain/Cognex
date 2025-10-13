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

inline alliance_t auto_alliance = alliance_t::unset;
inline field_side_t auto_side = field_side_t::unset;

inline std::string selected_auton = "";
