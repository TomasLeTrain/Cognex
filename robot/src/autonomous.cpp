#include "apis.h"
//

#include "autos.h"
#include "globals.h"
#include "main.h"
#include "screen/screen.h"
#include "systems/intake.h"
#include "systems/matchloader.h"

void testing_auton_func(){
	auto_side = field_side_t::right;
	auto_alliance = alliance_t::blue;
	simple_auton::run_auton();
}

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
		testing_auton_func();
    }
}
