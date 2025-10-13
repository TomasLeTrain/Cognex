#pragma once

#include "apis.h"
//

#include <functional>
#include <map>
#include <string>

#define NEW_AUTON(auton) \
    namespace auton {    \
    void run_auton();    \
    }

// make sure autons are defined in here AND in autonomous.cpp!!
NEW_AUTON(simple_auton)
NEW_AUTON(skills)
NEW_AUTON(skills2)
NEW_AUTON(simple_other_goal_auton)
NEW_AUTON(roboticon_skills)

// special disabled auton that does nothing
namespace disabled_auton {
inline void run_auton() {}
} // namespace disabled_auton

// maps the auton name to the corresponding function
extern std::map<std::string, std::function<void()>> auton_list;
