#include "main.h"

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");
}

// in the disabled state
void disabled() {}

// runs after initialize and before autonomous. Useful for auton selectors or other such code
void competition_initialize() {
}

void autonomous() {

}

void opcontrol() {
    while (true) {
        pros::lcd::print(0,"hello world!");
        pros::delay(10);
    }
}
