#include "apis.h"
//
#include "autons_list.h"

// clang-format off
std::unordered_map<std::string, std::function<void()>> auton_list = {
	// commented out ununsed autons to save on compile size

	// L_AUTON(roboticon_quals, "roboticon quals")
	//
	   L_AUTON(disabled_auton, "disabled")
	//
	// L_AUTON(skills, "old skills")
	//    L_AUTON(skills2, "new sklls")
	//    L_AUTON(simple_other_goal_auton, "simple other goal")
	//    L_AUTON(roboticon_skills, "roboticon skills")
	//
	// L_AUTON(eagles_skills, "eagles skills")

	// L_AUTON(max_skills, "max skills")

	
	// L_AUTON(sunshine_quals, "quals")
	L_AUTON(four_ball, "4 ball")
	//
	// L_AUTON(qual_match_first, "match first qual")
	// L_AUTON(sunshine_fast_auton, "fast auton")
	L_AUTON(awp, "awp")
	L_AUTON(elims_nineball_split, "elims nineball split")
	L_AUTON(easier_awp, "easier awp")
	L_AUTON(sunshine_quals, "seven split")
	L_AUTON(qual_match_first, "match first 7 split")
	L_AUTON(seven_ball,"seven ball")


	// L_AUTON(sunshine_skills, "skills")
	//
	// L_AUTON(sunshine_elims, "elims")

	L_AUTON(ninesix_skills, "skills")
};
// clang-format on
