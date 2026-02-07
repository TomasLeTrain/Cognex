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

void autoStateUpdate();
void setAlliance(alliance_t new_alliance);
void setFieldSide(field_side_t new_side);
void setAuton(std::string new_selected_auton);

alliance_t getAlliance();
field_side_t getFieldSide();
std::string getAuton();
