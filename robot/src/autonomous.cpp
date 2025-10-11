#include "autos.h"
#include "globals.h"
#include "liblvgl/core/lv_obj_pos.h"
#include "main.h"
#include "screen/screen.h"
#include "systems/intake.h"
#include "systems/matchloader.h"

#define AUTON(auton, name) { name, auton::run },

// clang-format off

// list of routines displayed in the auton selector - ALSO DEFINE IT IN
// autos.h!!
std::map<std::string, std::function<void()>> auton_list = {
    AUTON(disabled_auton, "disabled")

    AUTON(simple_auton, "simple")
	AUTON(skills, "old skills")
    AUTON(skills2, "new sklls")
    AUTON(simple_other_goal_auton, "simple other goal")
};

// clang-format on

void autonomous() {
    // initialize subsystems
    intake::init(false);
    matchloader::init(false);

    // change to W screen
    screen::setScreen(&screen::dvd_screen);

    if (selected_auton != "") {
        // the selected auton gets run
        auto selected_auton_function = auton_list[selected_auton];
        selected_auton_function();
    } else {
        // run some default auton - useful for testing
        // NOTE: select the disabled auton if you don't want anything to run!!!
        auto_side = field_side_t::right;
        auto_alliance = alliance_t::blue;
        simple_auton::run();
    }
}
