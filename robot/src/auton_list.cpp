#include "apis.h"
//
#include "autons_list.h"

// clang-format off
std::map<std::string, std::function<void()>> auton_list = {
    L_AUTON(disabled_auton, "disabled")

    L_AUTON(simple_auton, "simple")
	L_AUTON(skills, "old skills")
    L_AUTON(skills2, "new sklls")
    L_AUTON(simple_other_goal_auton, "simple other goal")
    L_AUTON(roboticon_skills, "current skills")
};
// clang-format on
