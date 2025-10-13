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
extern pros::Motor intake_motor;
extern pros::Motor score_motor;
extern pros::Motor bin_motor;

extern pros::Optical middle_intake_color_sensor;
extern pros::Optical bottom_intake_color_sensor;

// pistons
extern pros::adi::DigitalOut intake_recycle_piston;
extern pros::adi::DigitalOut intake_raise_piston;
extern pros::adi::DigitalOut matchloader_piston;

// odom rotation sensors
extern pros::Rotation sideways_odom_rotation;
extern pros::Rotation forwards_odom_rotation;

// particle filter distance sensors
extern pros::Distance front_distance;
extern pros::Distance back_distance;
extern pros::Distance left_distance;
extern pros::Distance right_distance;
