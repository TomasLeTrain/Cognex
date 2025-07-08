#include "main.h"
#include "systems/drivetrain.h"
#include "systems/intake.h"

void opcontrol(){
    // initialize tasks for each subsystem

    intake::init(true);
    
    // no need to initialize in auto
    base::init();

    while(true){
        // maybe unneeded?
        pros::delay(10);
    }
}
