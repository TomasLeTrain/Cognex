#pragma once

#include "apis.h"
//

#include <functional>
#include <map>
#include <string>

#define H_AUTON(auton) \
    namespace auton {  \
    void run_auton();  \
    }

#define L_AUTON(auton, name) { name, auton::run_auton },

// make sure autons are defined in here AND in autonomous.cpp!!
// H_AUTON(skills)
// H_AUTON(skills2)
// H_AUTON(simple_other_goal_auton)
//
// H_AUTON(roboticon_skills)
// H_AUTON(roboticon_quals)
//
// H_AUTON(seven_ball)
//
// H_AUTON(max_skills)
//
// H_AUTON(eagles_skills)
//
// H_AUTON(sunshine_quals)
// H_AUTON(sunshine_awp)
// H_AUTON(sunshine_skills)
// H_AUTON(sunshine_fast_auton)
// H_AUTON(qual_match_first)
//
// H_AUTON(sunshine_elims)

H_AUTON(awp)
H_AUTON(ninesix_skills)

// special disabled auton that does nothing
namespace disabled_auton {
inline void run_auton() {}
} // namespace disabled_auton

// maps the auton name to the corresponding function
extern std::unordered_map<std::string, std::function<void()>> auton_list;
