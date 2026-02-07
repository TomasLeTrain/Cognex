#include "apis.h"
//

#include "auton_globals.h"
#include "controller_ui/controller_auton_selector.h"
#include "globals.h"
#include "globals/device_globals.h"
#include "pros/misc.hpp"
#include <string>

namespace controller_ui {

void autonUpdate() {
    std::string line2 = getAuton();

    if (getAuton() == "") {
        line2 = "no selected auto!";
    }

    std::stringstream line3_stream;

    line3_stream << std::left << std::setfill(' ') << std::setw(10);
    if (getAlliance() == alliance_t::blue) {
        line3_stream << "blue";
    } else if (getAlliance() == alliance_t::red) {
        line3_stream << "red";
    } else {
        line3_stream << "none";
    }

    if (getFieldSide() == field_side_t::left) {
        line3_stream << "left";
    } else if (getFieldSide() == field_side_t::right) {
        line3_stream << "right";
    } else {
        line3_stream << "none";
    }

    // make sure both lines clear all content
    line2 += "                ";
    line3_stream << "                ";

    std::string line3 = line3_stream.str();

    // have to delay bewteen these updates
    controller.set_text(1, 0, line2);
    pros::delay(100);
    controller.set_text(2, 0, line3);
}

void general_update() {
	// TODO: add markers if there are any errors

    // update with battery information
    int capacity = (int)pros::battery::get_capacity();

    // controller.print(0, 0, ""line1);
    controller.print(0, 0, "capacity: %d%%", capacity);
    // pros::delay(200);
}

void init() {
    // clear
    controller.clear();

    pros::delay(100);
    // run an initial time
    autonUpdate();
    pros::delay(100);

    pros::Task([] {
        general_update();
        // controller updates don't have to be instant
        pros::delay(300);
    });
}
} // namespace controller_ui
