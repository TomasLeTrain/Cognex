#include "globals.h"
#include "main.h"
#include "autos.h"
#include "screen/screen.h"
#include "systems/intake.h"

#define AUTON(auton,name) {name, auton::run },

// list of routines displayed in the auton selector - ALSO DEFINE IT IN autos.h!!
std::map<std::string, std::function<void()>> auton_list = {
    AUTON(auton1,"auton1")
    AUTON(skills,"old skills")
    AUTON(skills2,"new sklls")
};


void autonomous(){
    // initialize subsystems
    intake::init(false);

    // if(auto_alliance == alliance_t::red){
    //     printf("selected red alliance\n");
    // }
    // if(auto_alliance == alliance_t::blue){
    //     printf("selected blue alliance\n");
    // }
    // if(auto_alliance == alliance_t::unset){
    //     printf("didn't set alliance!\n");
    // }
    //
    // if(auto_side == field_side_t::left){
    //     printf("selected left side\n");
    // }
    // if(auto_side == field_side_t::right){
    //     printf("selected right side\n");
    // }
    // if(auto_side == field_side_t::unset){
    //     printf("didn't set field side!\n");
    // }
    //
    // if(selected_auton != ""){
    //     printf("selected auton: %s\n",selected_auton.c_str());
    // }else{
    //     printf("didn't select auton \n");
    // }

    screen::setScreen(&screen::dvd_screen);

    // put the routine being worked on here - COMMENT OUT IN ACTUAL COMPETITION!!!
    // skills2::run();
    auton1::run();
    
    // COMMENT THIS IF NOT TESTING A SPECIFIC AUTON !!!!
    return;
    
    // dont run anything if no auton was selected
    if(selected_auton != ""){
        // the selected auton gets run
        auto selected_auton_function = auton_list[selected_auton];
        selected_auton_function();
    }
}
