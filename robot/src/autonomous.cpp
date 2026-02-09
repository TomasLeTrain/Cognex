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

    // selected_auton = "awp";
    // selected_auton = "elims nineball split";

    // selected_auton = "easier awp";

    // selected_auton = "quals";
    // selected_auton = "skills";
    // selected_auton = "fast auton";
    // selected_auton = "4 ball";
    //
    // selected_auton = "match first qual";
    // selected_auton = "elims";
    // selected_auton = "match first 7 split";
    // selected_auton = "seven ball";

    // selected_auton = "4 ball";
    //
    //
    // selected_auton = "match first 7 split";
    // selected_auton = "seven split";

    setFieldSide(field_side_t::left);
    setAlliance(alliance_t::red);
    setAuton("awp");
    setAuton("skills");

    // std::cout << "calling selected auto" << std::endl;
    auto selected_auton_function = auton_list[selected_auton];
    selected_auton_function();
}

void autonomous() {
    std::cout << "called ato" << std::endl;

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
