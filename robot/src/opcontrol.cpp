#include "main.h"
#include "systems/drivetrain.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "screen/screen.h"
#include "globals.h"
#include "autos.h"

void opcontrol(){
    // initialize tasks for each subsystem

    autonomous();
    return;

    intake::init(true);
    matchloader::init(true);
    
    // no need to initialize in auto
    base::init();

    screen::setScreen(&screen::dvd_screen);


    bool print_info = true;

    pros::Task smoother_task {[&] {
    while (print_info) {
        int start_time = pros::millis();
        printf(
          "start generation\nstart distances\nend distances\nstart " "parti" "cles" "\n");

        printf("%.1f %.1f %.1f\n",
               pf_motion_model.getPose().x.convert(in),
               pf_motion_model.getPose().y.convert(in),
               0.0);
        if(pf_model.getConfidence() != std::nullopt){
            printf("%.1f %.1f %.1f\n",
                    pf_model.getPose().x.convert(in),
                    pf_model.getPose().y.convert(in),
                    5.0);
        }
        printf("%.1f %.1f %.1f\n",
               smoother_model.getPose().x.convert(in),
               smoother_model.getPose().y.convert(in),
               10.0);

        printf(
          "end particles\ntotal weight: 0, time taken: 30000, " "timestamp:" " %d\n",
          start_time);
        printf("things done:1,1,0,%d\n",16384);
        printf("prediction:%.1f,%.1f,%.1f\n",
               smoother_model.getPose().x.convert(in),
               smoother_model.getPose().y.convert(in),
               smoother_model.getPose().orientation.convert(deg));
        printf("end generation\n");

        pros::delay(10);
    }
    }};

    RobotSetPose(-63_in, -16_in, 0);
    
    while(true){
        // maybe unneeded?
        pros::delay(10);
    }
}
