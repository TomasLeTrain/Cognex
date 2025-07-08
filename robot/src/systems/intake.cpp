#include "systems/intake.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "globals.h"

namespace intake {
// any needed variables can be specified here
// NOTE: it is recommended these variables are not changed directly by autos/other subsystems
// you should ideally provide functions to interact with these variables
bool is_driver = false;
bool tasks_active = false;
intake_state_t intake_state = disabled;
int motor_speed = 127;


// you can define any functions which could be used by autos/other subsystems by including them in the header file
void set(intake_state_t new_state, int speed){
    intake_state = new_state;
    motor_speed = std::abs(speed);
}


// used by intake specific functions - probably bad idea to expose (use set() in autos instead)
void toggle(){
    if(intake_state == disabled){
        intake_state = forwards;
    }else if(intake_state == forwards || intake_state == backwards){
        intake_state = disabled;
    }
}
void toggle_direction(){
    if(intake_state == forwards){
        intake_state = backwards;
    }else if(intake_state == backwards){
        intake_state = forwards;
    }else if(intake_state == disabled){
        // intake not on anyway
    }
}

// code that should run during driver
void driverUpdate() {
    // update motor state based on driver input
    int toggle_intake = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A);
    int toggle_intake_direction = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B);

    if(toggle_intake){
        toggle();
    }

    if(toggle_intake_direction){
        toggle_direction();
    }
}

// code that should run during autonomous
void autoUpdate() {
}

// updates the physical motor to match the current state of the subsystem
// runs regardless of driver mode
void motorUpdate(){
    if(intake_state == disabled){
        intake_motor.move(0);
    }else if(intake_state == forwards){
        intake_motor.move(motor_speed);
    }else if(intake_state == backwards){
        intake_motor.move(-motor_speed);
    }
}

// updates the state of the subsystem
void update() {
    if (is_driver) {
        driverUpdate();
    } else {
        autoUpdate();
    }
    motorUpdate();
}

void init(bool gdriver) {
    is_driver = gdriver;

    // don't make another task
    if (tasks_active) return;

    // run any code that should only occur once

    pros::Task main_intake_task([] {
        while (true) {
            update();
            pros::delay(10);
        }
    });

    tasks_active = true;
}
}; // namespace intake
