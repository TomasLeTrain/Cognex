#include "globals.h"
#include "pros/abstract_motor.hpp"
#include "vexmaps/odometry/tracking_wheel.hpp"
#include "vexmaps/smoother_model.hpp"

// motors use blue gears
auto motor_gearing = pros::MotorGears::blue;
// use motor encodings 
auto motor_encoding = pros::MotorEncoderUnits::rotations;

// vexmaps::LocalizationModel* pose_getter = &pf_motion_model;
vexmaps::LocalizationModel* pose_getter = &smoother_model;
vexmaps::LocalizationModel* orientation_getter = nullptr;


// motor groups
pros::MotorGroup left_motor_group ({-11, -14,  13}, motor_gearing, motor_encoding);
pros::MotorGroup right_motor_group({15,   16, -10}, motor_gearing,motor_encoding);

// inertial sensor
vexmaps::ScaledIMU imu(1, 363.0 / 360.0);

// intake motors
pros::Motor intake_motor(17);
pros::Motor score_motor(-7);
pros::Motor bin_motor(2);

pros::Optical middle_intake_color_sensor(3);
pros::Optical top_intake_color_sensor(8);


// pistons
pros::adi::DigitalOut intake_recycle_piston('A',false);
pros::adi::DigitalOut matchloader_piston('A',false);

// odom rotation sensors
pros::Rotation vertical_odom_rotation(20);
pros::Rotation horizontal_odom_rotation(5);


// particle filter distance sensors
pros::Distance front_distance(9);
pros::Distance back_distance(12);
pros::Distance left_distance(4);
pros::Distance right_distance(19);

/* vexmaps configuration */

// trackers and their offsets
// WARNING: The signs are OPPOSITE of lemlib's.
// you can use lemlib's tuning guide but have OPPOSITE signs!
Length horizontal_offset = 0.51_in;
Length vertical_offset = -0.31_in;

Length hor_odom_wheel_diameter = 1.995_in;
Length ver_odom_wheel_diameter = 2.0_in;

// if the tracker is not installed the list can be left empty -> tracker = {};
horizontalTrackers horizontal_trackers = {
    &horizontal_tracker
};
verticalTrackers vertical_trackers = {
    &vertical_tracker
};

// custom pf configs - probably can leave alone
vexmaps::MotionModelConfig motion_model_config = {};
// vexmaps::PFConfiguration Pfconfig = {.logging=true,.particle_logging=false};
vexmaps::PFConfiguration Pfconfig = {.logging=false,.particle_logging=false};
vexmaps::SmootherConfig smoother_config = {
};

// distance sensor offsets
units::V2Position front_distance_offsets = { 7_in, -3.59375_in };
units::V2Position left_distance_offsets = { 1_in, 4.75_in };
units::V2Position back_distance_offsets = { -7.125_in, 3.5_in };
units::V2Position right_distance_offsets = { 1_in, -4.75_in };

/* drivetrain / pid configuration */

// NOTE: remember to update every time the drivetrain changes!
drivetrain_config_t drivetrain_config {
    .track_width = 10.5,
    .wheel_diameter = 3.25,
    .rpm = 450,
    .horizontal_drift = 2,
};

lateral_pid_config_t lateral_pid_config {
    .P = 8,
    .I = 0,
    .D = 32,
    .anti_windup = 3,
    .small_error_range = 1.5,
    .small_error_range_timeout = 100,
    .large_error_range = 3,
    .large_error_range_timeout = 500,
    .maximum_accel = 85,
};

angular_pid_config_t angular_pid_config {
    .P = 3.6,
    .I = 0,
    .D = 30,
    .anti_windup = 3,
    .small_error_range = 3,
    .small_error_range_timeout = 100,
    .large_error_range = 5,
    .large_error_range_timeout = 500,
    .maximum_accel = 0,
};
