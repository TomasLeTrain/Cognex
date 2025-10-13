#include "autos.h"
#include "globals/vexmaps_globals.h"
#include "units/Vector2D.hpp"
#include <cmath>

// defaults
alliance_t auto_alliance = alliance_t::unset;
field_side_t auto_side = field_side_t::unset;

std::string selected_auton = "";

void RobotSetPose(double x, double y, double angle) {
    units::Pose pose = { x * in, y * in, angle * deg };

    if (orientation_getter != nullptr) {
        orientation_getter->setPose(pose);
    }
    pose_getter->setPose(pose);

    tracker.setPose(pose);
}

units::Pose RobotGetPose() {
    units::Pose pose = pose_getter->getPose();

    if (orientation_getter != nullptr) {
        pose.orientation = orientation_getter->getPose().orientation;
    }
    return pose;
}

void changePoseGetter(vexmaps::LocalizationModel* new_getter) {
    std::lock_guard lock(pose_mutex);
    pose_getter = new_getter;
}

// effectively resets to whatever mcl measures
void DistanceSensorReset(int timeout, double new_alpha) {
    // uses default config for all other values
    vexmaps::SmootherConfig new_config = smoother_config;

    // change alpha values to quickly reset to mcl pose
    new_config.pose_x = new_alpha;
    new_config.pose_y = new_alpha;

    smoother_model.changeConfiguration(new_config);
    pros::delay(timeout);
    smoother_model.changeConfiguration(smoother_config);
}

void DistanceSensorReset2(std::vector<laser_model_type*> enabled_lasers) {
    units::Pose current_pose = RobotGetPose();
    Angle theta = current_pose.orientation;

    // update all lasers
    for (auto laser : enabled_lasers) {
        laser->update(theta);
    }

    std::optional<Length> new_x = std::nullopt;
    std::optional<Length> new_y = std::nullopt;

    auto update_x = [&](laser_model_type* laser) {
        if (auto expected = laser->getExpected(); expected.has_value()) {
            // either set equal to or average both
            new_x = new_x ? (*new_x + expected->x) / 2 : Length(expected->x);
        }
    };
    auto update_y = [&](laser_model_type* laser) {
        if (auto expected = laser->getExpected(); expected.has_value()) {
            // either set equal to or average both
            new_y = new_y ? (*new_y + expected->y) / 2 : Length(expected->y);
        }
    };

    const Angle pi_2 = Angle(M_PI_2);
    const Angle pi = Angle(M_PI);
    const Angle pi3_2 = Angle(M_PI + M_PI_2);

    // either pointing left or right on global map
    if (units::abs(theta) <= 20_stDeg || units::abs(theta - pi) <= 20_stDeg) {
        for (auto laser : enabled_lasers) {
            if (laser == &left_laser_model || laser == &right_laser_model)
                update_y(laser);

            else if (laser == &front_laser_model || laser == &back_laser_model)
                update_x(laser);
        }
    }

    // either pointing upwards or downwards on global map
    if (units::abs(theta - pi_2) <= 20_stDeg ||
        units::abs(theta - pi3_2) <= 20_stDeg) {
        for (auto laser : enabled_lasers) {
            if (laser == &left_laser_model || laser == &right_laser_model)
                update_x(laser);

            else if (laser == &front_laser_model || laser == &back_laser_model)
                update_y(laser);
        }
    }
}
