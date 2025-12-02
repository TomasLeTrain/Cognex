#pragma once

#include "apis.h"
//
#include "auton_globals.h"
#include "globals/config.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"

using namespace blazing;

/*
 * blazing configs stuff
 *
 */

extern DifferentialDrivetrain drivetrain;

extern ForwardsTracker left_motor_tracker;
extern ForwardsTracker right_motor_tracker;

extern ForwardsTracker forwards_tracker;
extern SidewaysTracker sideways_tracker;
extern TrackingImu imu_tracker;

extern ArcOdomTracker tracker;

// controller stuff
extern PID<Length, Voltage> linear_pid;

extern PID<Angle, Voltage> angular_pid;

extern LinearSlewController linear_slew;
extern AngularSlewController angular_slew;
extern LinearSlewController driver_linear_slew;

extern LinearVoltageClampController linear_voltage_constraints;
extern AngularVoltageClampController angular_voltage_constraints;

extern PIDLinearController linear_pid_controller;
extern PIDAngularController angular_pid_controller;

// velocity control stuff
extern blazing::lyfast::VelocityController velocity_controller;
extern lyfast::VelocityFeedforward<decltype(velocity_controller)>
  controller_velocity_controller;

extern Controllers<decltype(linear_pid_controller),
                   decltype(angular_pid_controller),
                   decltype(controller_velocity_controller),
                   decltype(linear_slew),
                   decltype(angular_slew),
                   decltype(linear_voltage_constraints),
                   decltype(angular_voltage_constraints)>
  controllers;

// normal tolerances
extern Tolerances<decltype(linear_tolerances_config.error),
                  decltype(linear_tolerances_config.velocity)>
  linearTolerances;

extern Tolerances<decltype(angular_tolerances_config.error),
                  decltype(angular_tolerances_config.velocity)>
  angularTolerances;

// large tolerances
extern Tolerances<decltype(linear_tolerances_config.large_error),
                  decltype(linear_tolerances_config.large_velocity)>
  largeLinearTolerances;

extern Tolerances<decltype(angular_tolerances_config.large_error),
                  decltype(angular_tolerances_config.large_velocity)>
  largeAngularTolerances;

// chain tolerances
extern Tolerances<decltype(linear_tolerances_config.chain_error)>
  chainLinearTolerances;

extern Tolerances<decltype(angular_tolerances_config.chain_error)>
  chainAngularTolerances;

extern normalLargeChainTolerances<decltype(linearTolerances),
                                  decltype(angularTolerances),
                                  decltype(largeLinearTolerances),
                                  decltype(largeAngularTolerances),
                                  decltype(chainLinearTolerances),
                                  decltype(chainAngularTolerances)>
  tolerances;

extern Chassis<decltype(drivetrain), decltype(tracker), decltype(tolerances)>
  chassis;

// executors
extern RunExecutor run;
extern AsyncExecutor async_exec;

extern MotionBuilder<decltype(chassis), decltype(controllers)> mb_blazing;

extern Chassis<decltype(drivetrain),
               decltype(vexmaps_tracker),
               decltype(tolerances)>
  vexmaps_chassis;

extern MotionBuilder<decltype(vexmaps_chassis), decltype(controllers)> mb;

extern ChainedExecutor chain;

// motion things

// custom cos-like func
double angular_linear_func(Angle angle);
