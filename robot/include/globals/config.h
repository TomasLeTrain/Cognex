#pragma once

#include "apis.h"
//

// easier use of libs
using namespace blazing;
using namespace vexmaps;

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
    ErrorTolerance<T> error = T(0);
    VelocityTolerance<T> velocity = T(100000) / sec;

    Time large_duration = 1000_sec;
    ErrorTolerance<T> large_error = T(0);
    VelocityTolerance<T> large_velocity = T(100000) / sec;

    Time chain_duration = 1000_sec;
    ErrorTolerance<T> chain_error = T(0);
    VelocityTolerance<T> chain_velocity = T(100000) / sec;
};

extern drivetrain_config_t drivetrain_config;

extern linear_pid_config_t linear_pid_config;
extern angular_pid_config_t angular_pid_config;

extern tolerances_config_t<Length> linear_tolerances_config;
extern tolerances_config_t<Angle> angular_tolerances_config;

// tracker configs
extern tracker_config_t sideways_tracker_config;
extern tracker_config_t forwards_tracker_config;

// different vexmaps configurations
extern vexmaps::MotionModelConfig motion_model_config;
extern vexmaps::PFConfiguration Pfconfig;
extern vexmaps::SmootherConfig smoother_config;

// header only configs

// likely does not need to change
struct CustomDistanceSensorConfiguration {
    // all floats without units are in meters
    static constexpr double exp_l = 1.5;
    static constexpr double std_deviation = (2_in).internal();

    // sum of coefficients 1
    static constexpr double randomCoeff = 0.125;
    static constexpr double expCoeff = 0.075;
    static constexpr double normalCoeff = 0.8;

    static constexpr bool logging = false;
};

// only variable which cannot be set on globals.cpp
// constexpr size_t pf_particle_count = 10000;
constexpr size_t pf_particle_count = 500;


