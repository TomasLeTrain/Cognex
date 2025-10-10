#pragma once

#include <map>
#include <string>
#include <functional>

#define NEW_AUTON(auton) \
    namespace auton {    \
    void run();          \
    }

// make sure autons are defined in here AND in autonomous.cpp!!
NEW_AUTON(simple_auton)
NEW_AUTON(skills)
NEW_AUTON(skills2)
NEW_AUTON(simple_other_goal_auton)

// maps the auton name to the corresponding function
extern std::map<std::string, std::function<void()>> auton_list;
