#include "main.h"
#include "systems/drivetrain.h"

void opcontrol(){
    // initialize tasks for each subsystem
    

    pros::Task drivebase_task([] {
        while (true) {
            base::driveUpdate();
            pros::delay(10);
        }
    });

    // while(true){
    //     // uncomplicated functions which do not really require their own tasks
    //     pros::delay(20);
    // }
}
