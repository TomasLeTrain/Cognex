#pragma once

#include "apis.h"
#include "blazing/tolerances.hpp"
#include "vexmaps/mcl/config.hpp"

//

// structs for helping specify configs
struct drivetrain_config_t {
    Length track_width;
    Length wheel_diameter;
    AngularVelocity rpm;
};

struct linear_pid_config_t {
    double kp;
    double ki;
    double kd;
    std::optional<double> windupRange = std::nullopt;
    std::optional<double> maxVoltage = 127;
    // same as trusting fully
    std::optional<double> derivative_alpha = std::nullopt;

    Time timeUnits = 50_msec;
    Length inputUnits = 1_in;
    Voltage outputUnits = Voltage(1.0 / 127.0);
};

struct angular_pid_config_t {
    double kp;
    double ki;
    double kd;
    std::optional<double> windupRange = std::nullopt;
    std::optional<double> maxVoltage = 127;
    // same as trusting fully
    std::optional<double> derivative_alpha = std::nullopt;
    Time timeUnits = 50_msec;
    Angle inputUnits = 1_stDeg;
    Voltage outputUnits = Voltage(1.0 / 127.0);
};

struct tracker_config_t {
    Length diameter;
    Length offset;
};

// default exits are made to never trigger unless error is set
template<typename T>
struct tolerances_config_t {
    Time duration = 1000_sec;
    blazing::ErrorTolerance<T> error = T(0);
    blazing::VelocityTolerance<T> velocity = T(100000) / sec;
    blazing::HalfCircleTolerance halfCircle { std::nullopt };

    Time large_duration = 1000_sec;
    blazing::ErrorTolerance<T> large_error = T(0);
    blazing::VelocityTolerance<T> large_velocity = T(100000) / sec;
    blazing::HalfCircleTolerance large_halfCircle { std::nullopt };

    Time chain_duration = 1000_sec;
    blazing::ErrorTolerance<T> chain_error = T(0);
    blazing::VelocityTolerance<T> chain_velocity = T(100000) / sec;
    blazing::HalfCircleTolerance chain_halfCircle { std::nullopt };
};

extern drivetrain_config_t drivetrain_config;

extern linear_pid_config_t linear_pid_config;
extern angular_pid_config_t angular_pid_config;

extern tolerances_config_t<Length> linear_tolerances_config;
extern tolerances_config_t<Angle> angular_tolerances_config;

// tracker configs
extern tracker_config_t sideways_tracker_config;
extern tracker_config_t forwards_tracker_config;

// particle filter distance sensors
extern pros::Distance front_distance;
extern pros::Distance back_distance;
extern pros::Distance left_distance;
extern pros::Distance right_distance;

// different vexmaps configurations
extern vexmaps::MotionModelConfig motion_model_config;
extern vexmaps::PFConfiguration Pfconfig;
extern vexmaps::SmootherConfig smoother_config;

extern vexmaps::DistanceSensorConfig distance_sensor_config;

// header only configs

// only variable which cannot be set on globals.cpp
// constexpr size_t pf_particle_count = 10000;
constexpr size_t pf_particle_count = 500;
