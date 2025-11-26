#include "apis.h"
//
#include "autons_list.h"

// clang-format off
std::unordered_map<std::string, std::function<void()>> auton_list = {
	// commented out ununsed autons to save on compile size

	// L_AUTON(roboticon_quals, "roboticon quals")
	//
	//    L_AUTON(disabled_auton, "disabled")
	//
	//    L_AUTON(simple_auton, "simple")
	// L_AUTON(skills, "old skills")
	//    L_AUTON(skills2, "new sklls")
	//    L_AUTON(simple_other_goal_auton, "simple other goal")
	//    L_AUTON(roboticon_skills, "roboticon skills")
	//
	// L_AUTON(eagles_skills, "eagles skills")
	// L_AUTON(eagles_quals, "eagles quals")

	L_AUTON(sunshine_skills, "current skills")

};
// clang-format on
