#include "globals.h"
#include "pros/abstract_motor.hpp"

// motors use blue gears
auto motor_gearing = pros::MotorGears::blue;
// use motor encodings 
auto motor_encoding = pros::MotorEncoderUnits::rotations;

// motor groups
pros::MotorGroup left_motor_group ({1,2,3}, motor_gearing, motor_encoding);
pros::MotorGroup right_motor_group({4,5,6}, motor_gearing,motor_encoding);

// inertial sensor
pros::Imu imu(3);

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


// lemlib/pid options
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
