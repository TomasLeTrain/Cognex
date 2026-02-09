#include "apis.h"
//
#include "auton_globals.h"
#include "controller_ui/controller_auton_selector.h"
#include "screen/screen.h"

// called every time
void autoStateUpdate() {
    // update
    screen::auton_select::ui_update();
    controller_ui::autonUpdate();
}

void setAlliance(alliance_t new_alliance) {
    auto_alliance = new_alliance;
    // std::cout << "changed auton to "
    //           << (new_alliance == alliance_t::red ? "red" : "blue")
    //           << std::endl;
    autoStateUpdate();
}

void setFieldSide(field_side_t new_side) {
    auto_side = new_side;
    // std::cout << "changed auton to "
    //           << (new_side == field_side_t::right ? "right" : "left")
    //           << std::endl;
    autoStateUpdate();
}

void setAuton(std::string new_selected_auton) {
    selected_auton = new_selected_auton;
    autoStateUpdate();
    // std::cout << "changed auton to " << new_selected_auton << std::endl;
}

alliance_t getAlliance() {
    return auto_alliance;
}

field_side_t getFieldSide() {
    return auto_side;
}

std::string getAuton() {
    return selected_auton;
}
