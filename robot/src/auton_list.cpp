#include "apis.h"
//
#include "autons_definitions.h"
#include "autons_list.h"

// clang-format off
std::unordered_map<std::string, std::function<void()>> auton_list = {
	// L_AUTON(four_ball, "4 ball")
	// L_AUTON(elims_nineball_split, "elims nineball split")
	L_AUTON(awp, "awp")
	//
	// L_AUTON(seven_split, "seven split")
	// L_AUTON(qual_match_first, "match first 7 split")
	// L_AUTON(seven_ball,"seven ball")
	//
	L_AUTON(skills,"skills")
};
// clang-format on
