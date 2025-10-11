#pragma once

// defines all the autons
#include "autons_list.h"

// helpful includes for autons
#include "apis.h"
#include "globals.h"
#include "units/Angle.hpp"

/* auton utils - leave alone */

// set pose of the robot - uses lemlib coordinate system
inline void RobotSetPose(double x, double y, double angle) {
    units::Pose pose = { x * in, y * in, angle * deg };

    if (orientation_getter != nullptr) {
        orientation_getter->setPose(pose);
    }
    pose_getter->setPose(pose);

    // update lemlib pose immediately to be able to run motions immediately
    tracker.setPose(pose);
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

// effectively resets to whatever mcl measures
inline void DistanceSensorReset(int timeout = 150, double new_alpha = 0.8) {
    // uses default config for all other values
    vexmaps::SmootherConfig new_config = smoother_config;

    // change alpha values to quickly reset to mcl pose
    new_config.pose_x = new_alpha;
    new_config.pose_y = new_alpha;

    smoother_model.changeConfiguration(new_config);
    pros::delay(timeout);
    smoother_model.changeConfiguration(smoother_config);
}
