#include "main.h"
#include "autos.h"
#include "systems/intake.h"

void autonomous(){
    // initialize subsystems
    intake::init(false);

    // no selector for now
    // auton1::run();
    skills::run();
}
