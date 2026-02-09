#include "apis.h"
//

#include "auton_globals.h"
#include "controller_ui/controller_auton_selector.h"
#include "globals.h"
#include "globals/device_globals.h"
#include "pros/misc.hpp"
#include <string>

namespace controller_ui {

bool warning_triggered = false;
bool critical_error_triggered = false;
bool was_connected = false;
bool need_auton_update = false;

void autonUpdate() {
    need_auton_update = true;
}

void actualAutonUpdate() {
    std::string line2 = getAuton();

    if (getAuton() == "") {
        line2 = "no selected auto!";
    }

    std::stringstream line3_stream;

    line3_stream << std::left << std::setfill(' ') << std::setw(9);
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

    if (critical_error_triggered || warning_triggered) {
        line3_stream << std::right << std::setfill(' ') << std::setw(6);
        std::string symbols = "";
        if (critical_error_triggered) symbols += "E";
        if (warning_triggered) symbols += "w";
        line3_stream << symbols;
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

void criticalErrorTriggered() {
    critical_error_triggered = true;
    autonUpdate();
}

void warningTriggered() {
    warning_triggered = true;
    autonUpdate();
}

void general_update() {
    // TODO: add markers if there are any errors

    // controller not co
    if (was_connected && !controller.is_connected()) {
        // controller dc'd
    }

    if (!was_connected && controller.is_connected()) {
        // controller connected, need to update its state?
        controller.clear();
        pros::delay(100);
        // autonUpdate();
        // pros::delay(300);
    }

    // update with battery information
    int capacity = (int)pros::battery::get_capacity();

    // controller.print(0, 0, ""line1);
    controller.print(0, 0, "capacity: %d%%", capacity);

    if (need_auton_update) {
        pros::delay(100);
        actualAutonUpdate();

        need_auton_update = false;
    }

    was_connected = controller.is_connected();

    // pros::delay(200);
}

void init() {
    // initla clear
    // controller.clear();

    // pros::delay(100);
    // // run an initial time
    // autonUpdate();
    // pros::delay(100);

    pros::Task([] {
        general_update();
        // controller updates don't have to be instant
        pros::delay(300);
    });
}
} // namespace controller_ui
