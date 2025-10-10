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
#include "units/all.hpp"
#include "vexmaps/api.hpp"
#include "vexmaps/smoother_model.hpp"

#define NEW_AUTON(auton) \
    namespace auton {    \
    void run();          \
    }
#define NEW_AUTONS(args...) NEW_AUTON(args)()
// #define AUTON(auton) {#auton, auton::run },

// make sure autons are defined in here AND in autonomous.cpp!!
NEW_AUTON(simple_auton)
NEW_AUTON(skills)
NEW_AUTON(skills2)
NEW_AUTON(simple_other_goal_auton)

extern std::map<std::string, std::function<void()>> auton_list;

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

inline void DistanceSensorReset(int timeout = 150, double new_alpha = 0.8) {
    vexmaps::SmootherConfig new_config = smoother_config;

    // change alpha values to quickly reset to mcl pose
    new_config.alpha_x = new_alpha;
    new_config.alpha_y = new_alpha;

    smoother_model.changeConfiguration(new_config);
    pros::delay(timeout);
    smoother_model.changeConfiguration(smoother_config);
}
