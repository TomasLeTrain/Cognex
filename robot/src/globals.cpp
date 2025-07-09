#include "globals.h"
#include "pros/abstract_motor.hpp"
#include "vexmaps/odometry/tracking_wheel.hpp"
#include "vexmaps/smoother_model.hpp"

// motors use blue gears
auto motor_gearing = pros::MotorGears::blue;
// use motor encodings 
auto motor_encoding = pros::MotorEncoderUnits::rotations;

// motor groups
pros::MotorGroup left_motor_group ({-3,10,-2}, motor_gearing, motor_encoding);
pros::MotorGroup right_motor_group({1,-9,4}, motor_gearing,motor_encoding);

// inertial sensor
vexmaps::ScaledIMU imu(13, 363.0 / 360.0);

// intake motor/s?
pros::Motor intake_motor(15);

// pistons
pros::adi::DigitalOut matchloader_piston('A',false);

// odom rotation sensors
pros::Rotation vertical_odom_rotation(-7);
pros::Rotation horizontal_odom_rotation(-12);


// particle filter distance sensors
pros::Distance front_distance(6);
pros::Distance back_distance(5);
pros::Distance left_distance(16);
pros::Distance right_distance(20);

/* vexmaps configuration */

// trackers and their offsets
// WARNING: The signs are OPPOSITE of lemlib's.
// you can use lemlib's tuning guide but have OPPOSITE signs!
Length horizontal_offset = 0.7_in;
Length vertical_offset = 0.525_in;

Length odom_wheel_diameter = 1.995_in;

// if the tracker is not installed the list can be left empty -> tracker = {};
horizontalTrackers horizontal_trackers = {
    &horizontal_tracker
};
verticalTrackers vertical_trackers = {
    &vertical_tracker
};

// custom pf configs - probably can leave alone
vexmaps::MotionModelConfig motion_model_config = {};
vexmaps::PFConfiguration Pfconfig = {.logging=true,.particle_logging=true};
vexmaps::SmootherConfig smoother_config = {};

// distance sensor offsets
units::V2Position front_distance_offsets = { 5.25_in, 5.4375_in };
units::V2Position left_distance_offsets = { 3_in, 5.25_in };
units::V2Position back_distance_offsets = { -4_in, -1.84375_in };
units::V2Position right_distance_offsets = { 4.25_in, -5.375_in };

// vexmaps::LocalizationModel* pose_getter = &smoother_model;
vexmaps::LocalizationModel* pose_getter = &smoother_model;
vexmaps::LocalizationModel* orientation_getter = nullptr;


/* drivetrain / pid configuration */

// NOTE: remember to update every time the drivetrain changes!
drivetrain_config_t drivetrain_config {
    .track_width = 10.5,
    .wheel_diameter = 2.75,
    .rpm = 480,
    .horizontal_drift = 0,
};

lateral_pid_config_t lateral_pid_config {
    .P = 8,
    .I = 0,
    .D = 32,
    .anti_windup = 3,
    .small_error_range = 1,
    .small_error_range_timeout = 100,
    .large_error_range = 3,
    .large_error_range_timeout = 500,
    .maximum_accel = 87,
};

angular_pid_config_t angular_pid_config {
    .P = 3.6,
    .I = 0,
    .D = 30,
    .anti_windup = 3,
    .small_error_range = 1,
    .small_error_range_timeout = 100,
    .large_error_range = 3,
    .large_error_range_timeout = 500,
    .maximum_accel = 0,
};
