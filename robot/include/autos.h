#pragma once

// defines all the autons
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

// helpful includes for autons
#include "api.h"
#include "globals.h"
#include "lemlib/api.hpp"
#include "vexmaps/api.hpp"
#include "units/all.hpp"

#define NEW_AUTON(auton) namespace auton{ void run(); }
#define NEW_AUTONS(args...) NEW_AUTON(args)()
// #define AUTON(auton) {#auton, auton::run },

// make sure autons are defined in here AND in autonomous.cpp!!
NEW_AUTON(auton1)
NEW_AUTON(skills)
NEW_AUTON(skills2)

extern std::map<std::string, std::function<void()>> auton_list;


/* auton utils - leave alone */

// set pose of the robot - uses lemlib coordinate system
inline void RobotSetPose(Length x, Length y, float angle){
    Angle orientation = from_cDeg(angle);

    units::Pose pose = {x,y,orientation};

    if(orientation_getter != nullptr){
        orientation_getter->setPose(pose);
    }
    pose_getter->setPose(pose);
}


inline units::Pose RobotGetPose(){
    units::Pose pose = pose_getter->getPose();

    if(orientation_getter != nullptr){
        pose.orientation = orientation_getter->getPose().orientation;
    }
    return pose;
}
