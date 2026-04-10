#include "apis.h"
//

#include "auton_globals.h"
#include "autos.h"
#include "globals.h"
#include "main.h"
#include "screen/screen.h"
#include "systems/drivetrain.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/wings.h"

// run if no auton is selected - useful for testing
// NOTE: select the disabled auton if you don't want anything to run!!!
void testing_auton_func() {
    setFieldSide(field_side_t::left);
    setAlliance(alliance_t::red);

    setAuton("awp");

    auto selected_auton_function = auton_list[selected_auton];
    selected_auton_function();
}

void autonomous() {
    std::cout << "called a8to" << std::endl;

    // initialize subsystems
    intake::init(false);
    matchloader::init(false);
    wings::init(false);
    odom_retract::init(false);

    std::cout << "init everything" << std::endl;

    // change to W screen
    // screen::setScreen(&screen::bouncing_dvd_screen::screen);

    if (selected_auton != "") {
        // sets dvd screen only if on match
        screen::setScreen(&screen::bouncing_dvd_screen::screen);

        // the selected auton gets run
        auto selected_auton_function = auton_list[selected_auton];
        selected_auton_function();
    } else {
        std::cout << "callign testin" << std::endl;
        testing_auton_func();
    }
}
