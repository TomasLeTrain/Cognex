#pragma once

#include "apis.h"
//
#include "globals/config.h"

/*
 * Hardware configuration
 * add any hardware here and define it in globals.cpp as well
 * */

// controller - can probably leave alone forever
extern pros::Controller controller;

// motor groups
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;

// inertial sensor
extern vexmaps::ScaledIMU imu;

// intake motors
extern pros::Motor bottom_motor;
extern pros::Motor top_motor;

extern pros::Optical middle_intake_color_sensor;
extern pros::Optical bottom_intake_color_sensor;

// pistons
extern pros::adi::DigitalOut top_intake_piston;
extern pros::adi::DigitalOut middle_intake_piston;
extern pros::adi::DigitalOut matchloader_piston;
extern pros::adi::DigitalOut wings_piston;
extern pros::adi::DigitalOut odom_retract_piston;
extern pros::adi::DigitalOut bottom_intake_piston;

// odom rotation sensors
extern pros::Rotation sideways_odom_rotation;
extern pros::Rotation forwards_odom_rotation;
