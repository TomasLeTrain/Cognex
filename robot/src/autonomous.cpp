#include "apis.h"
//

#include "auton_globals.h"
#include "autos.h"
#include "globals.h"
#include "main.h"
#include "screen/screen.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"

// run if no auton is selected - useful for testing
// NOTE: select the disabled auton if you don't want anything to run!!!
void testing_auton_func() {
    auto_side = field_side_t::right;
    auto_alliance = alliance_t::red;
    // simple_auton::run_auton();
	// printf("got before selected auto\n");

    selected_auton = "current skills";
    // selected_auton = "current quals";
    // selected_auton = "current skills";
    auto selected_auton_function = auton_list[selected_auton];
	// printf("got after selected auto\n");
    selected_auton_function();
}

void autonomous() {
    // initialize subsystems
    intake::init(false);
    matchloader::init(false);
    wings::init(false);
	odom_retract::init(false);

    // change to W screen
    // screen::setScreen(&screen::bouncing_dvd_screen::screen);

    if (selected_auton != "") {
        // the selected auton gets run
        auto selected_auton_function = auton_list[selected_auton];
        selected_auton_function();

		// sets dvd screen only if on match
		screen::setScreen(&screen::bouncing_dvd_screen::screen);
    } else {
        testing_auton_func();
    }
}
