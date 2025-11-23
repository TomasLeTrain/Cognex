#include "autos.h"
#include "globals/vexmaps_globals.h"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include <cmath>

// defaults
alliance_t auto_alliance = alliance_t::unset;
field_side_t auto_side = field_side_t::unset;

std::string selected_auton = "";

void RobotSetPose(units::Pose pose) {
    model_manager.setPose(pose);

    tracker.setPose(pose);
}

void RobotSetPose(double x, double y, double angle) {
    RobotSetPose({ x * in, y * in, angle * deg });
}

units::Pose RobotGetPose() {
    // return model_manager.getPose();
    return { tracker.getPosition(), tracker.getAngle() };
}

// effectively resets to whatever mcl measures
void DistanceSensorReset(int timeout, double new_alpha) {
    // uses default config for all other values
    // vexmaps::SmootherConfig new_config = smoother_config;
    //
    // // change alpha values to quickly reset to mcl pose
    // new_config.alpha_x = new_alpha;
    // new_config.alpha_y = new_alpha;
    //
    // smoother_model.changeConfiguration(new_config);
    // pros::delay(timeout);
    // smoother_model.changeConfiguration(smoother_config);
}

void LaserResets(std::vector<laser_model_type*> enabled_lasers,
                 bool x,
                 bool y) {
    units::Pose current_pose = RobotGetPose();
    Angle theta = current_pose.orientation;
    theta = units::constrainAngle2pi(theta);

    // update all lasers
    for (auto laser : enabled_lasers) {
        laser->update(theta,std::nullopt);
        std::cout << "updated laser!\n";
    }

    std::optional<Length> new_x = std::nullopt;
    std::optional<Length> new_y = std::nullopt;

    auto update_x = [&](laser_model_type* laser) {
        if (!x) return;
        auto expected = laser->getExpected();
        if (expected.has_value()) {
            std::cout << "x: has expected: " << expected.value().x.convert(in)
                      << " " << expected.value().y.convert(in) << std::endl;
            // either set equal to or average both
            new_x = new_x ? (*new_x + expected->x) / 2 : Length(expected->x);
        }
    };
    auto update_y = [&](laser_model_type* laser) {
        if (!y) return;
        auto expected = laser->getExpected();
        if (expected.has_value()) {
            std::cout << "y: has expected: " << expected.value().x.convert(in)
                      << " " << expected.value().y.convert(in) << std::endl;
            // either set equal to or average both
            new_y = new_y ? (*new_y + expected->y) / 2 : Length(expected->y);
        }
    };

    // either pointing left or right on global map
    if (units::abs(units::cos(theta)) >= M_SQRT1_2) {
        std::cout << "pointing left/right" << std::endl;
        for (auto laser : enabled_lasers) {
            if (laser == &left_laser_model || laser == &right_laser_model)
                update_y(laser);

            else if (laser == &front_laser_model || laser == &back_laser_model)
                update_x(laser);
        }
    }

    // either pointing upwards or downwards on global map
    if (units::abs(units::sin(theta)) >= M_SQRT1_2) {
        std::cout << "up/down" << std::endl;
        for (auto laser : enabled_lasers) {
            if (laser == &left_laser_model || laser == &right_laser_model)
                update_x(laser);

            else if (laser == &front_laser_model || laser == &back_laser_model)
                update_y(laser);
        }
    }

    // set to new coordinates
    RobotSetPose({ new_x.value_or(current_pose.x),
                   new_y.value_or(current_pose.y),
                   theta });
}
