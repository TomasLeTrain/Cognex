#pragma once

// defines all the autons
#include "autons_list.h"

// helpful includes for autons
#include "api.h"
#include "globals.h"
#include "lemlib/api.hpp"
#include "units/all.hpp"
#include "vexmaps/api.hpp"

/* auton utils - leave alone */

// set pose of the robot - uses lemlib coordinate system
inline void RobotSetPose(Length x, Length y, float angle) {
    Angle orientation = from_cDeg(angle);

    units::Pose pose = { x, y, orientation };

    if (orientation_getter != nullptr) {
        orientation_getter->setPose(pose);
    }
    pose_getter->setPose(pose);

    // update lemlib pose immediately to be able to run motions immediately
    chassis.setPose(to_in(x), to_in(y), angle);
}

inline units::Pose RobotGetPose() {
    units::Pose pose = pose_getter->getPose();

    if (orientation_getter != nullptr) {
        pose.orientation = orientation_getter->getPose().orientation;
    }
    return pose;
}

inline void changePoseGetter(vexmaps::LocalizationModel* new_getter) {
    std::lock_guard lock(pose_mutex);
    pose_getter = new_getter;
}
