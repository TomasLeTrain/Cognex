#include "globals.h"
#include "pros/abstract_motor.hpp"
#include "vexmaps/odometry/tracking_wheel.hpp"
#include "vexmaps/smoother_model.hpp"

// motors use blue gears
auto motor_gearing = pros::MotorGears::blue;
// use motor encodings 
auto motor_encoding = pros::MotorEncoderUnits::rotations;

// motor groups
pros::MotorGroup left_motor_group ({1,2,3}, motor_gearing, motor_encoding);
pros::MotorGroup right_motor_group({4,5,6}, motor_gearing,motor_encoding);

// inertial sensor
vexmaps::ScaledIMU imu(3);

// intake motor/s?
pros::Motor intake_motor(3);

// pistons
pros::adi::DigitalOut matchloader_piston('A',false);

// odom rotation sensors
pros::Rotation horizontal_odom_rotation(1);
pros::Rotation vertical_odom_rotation(3);

// particle filter distance sensors
pros::Distance front_distance(1);
pros::Distance back_distance(3);
pros::Distance left_distance(4);
pros::Distance right_distance(5);


/* vexmaps configuration */

// trackers and their offsets
// WARNING: The signs are OPPOSITE of lemlib's.
// you can use lemlib's tuning guide but have OPPOSITE signs!
Length horizontal_offset = 1_in;
Length vertical_offset = 1_in;

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
vexmaps::PFConfiguration Pfconfig = {};
vexmaps::SmootherConfig smoother_config = {};

// distance sensor offsets
units::V2Position front_distance_offsets = { 5.25_in, 5.4375_in };
units::V2Position left_distance_offsets = { 3_in, 5.25_in };
units::V2Position back_distance_offsets = { -4_in, -1.84375_in };
units::V2Position right_distance_offsets = { 4.25_in, -5.375_in };

/* drivetrain / pid configuration */

// NOTE: remember to update every time the drivetrain changes!
drivetrain_config_t drivetrain_config {
    .track_width = 10,
    .wheel_diameter = 3.25,
    .rpm = 450,
    .horizontal_drift = 0,
};

lateral_pid_config_t lateral_pid_config {
    .P = 1,
    .I = 1,
    .D = 1,
    .anti_windup = 1,
    .small_error_range = 1,
    .small_error_range_timeout = 1,
    .large_error_range = 1,
    .large_error_range_timeout = 1,
    .maximum_accel = 1,
};

angular_pid_config_t angular_pid_config {
    .P = 1,
    .I = 1,
    .D = 1,
    .anti_windup = 1,
    .small_error_range = 1,
    .small_error_range_timeout = 1,
    .large_error_range = 1,
    .large_error_range_timeout = 1,
    .maximum_accel = 1,
};
