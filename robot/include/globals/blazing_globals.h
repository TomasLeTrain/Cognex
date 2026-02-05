#pragma once

#include "apis.h"
//
#include "auton_globals.h"
#include "blazing/controllers/clamp.hpp"
#include "blazing/controllers/controllers.hpp"
#include "globals/config.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
#include "lyfast/vel_controller.hpp"
#include "units/Angle.hpp"

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

extern PID<Angle, Voltage> turn_drive_pid;
extern PID<Angle, Voltage> turn_heading_pid;

extern LinearSlewController linear_slew;
extern AngularSlewController angular_slew;
// extern LinearSlewController driver_linear_slew;

extern LinearVoltageClampController linear_voltage_constraints;
extern AngularVoltageClampController angular_voltage_constraints;

extern PIDLinearController linear_pid_controller;
extern PIDAngularController angular_pid_controller;

extern blazing::lyfast::DifferentialVelocityController
  linear_velocity_controller;
extern blazing::lyfast::DifferentialVelocityController
  angular_velocity_controller;

// used exclusively for turning
extern lyfast::ArcadeVelocityController turn_vel_controller;

extern lyfast::ArcadeVelocityController vel_controller;

extern lyfast::VelocityFeedforward<decltype(vel_controller)>
  controller_velocity_controller;

// start linear velocity stuff //
extern PID<Length, LinearVelocity> linear_vel_pid;
extern PIDLinearVelocityController linear_vel_pid_controller;

// linear mp stuff
extern lyfast::mpFeedback<Length> linear_mp_feedback;
extern LinearVelocityFeedbackController<decltype(linear_mp_feedback)>
  linear_mp_feedback_controller;

extern LinearVelocitySlewController linear_vel_slew_controller;
extern LinearVelocityClampController linear_vel_clamp_controller;

// end linear velocity stuff //

// start angular velocity stuff //
extern PID<Angle, AngularVelocity> linear_angular_vel_pid;
extern PID<Angle, AngularVelocity> turn_heading_vel_pid;

extern PIDAngularVelocityController angular_vel_pid_controller;

extern AngularVelocitySlewController angular_vel_slew_controller;
extern AngularVelocityClampController angular_vel_clamp_controller;

// lateral controllers
extern PID<Length, Voltage> lateral_pid;
extern PID<Length, AngularVelocity> lateral_vel_pid;

extern LateralVelocityFeedbackController<decltype(lateral_vel_pid)>
  lateral_vel_controller;
extern LateralFeedbackController<decltype(lateral_pid)> lateral_controller;

// end angular velocity stuff //

extern Controllers<decltype(linear_pid_controller),
                   decltype(angular_pid_controller),
                   decltype(controller_velocity_controller),
                   decltype(linear_slew),
                   decltype(angular_slew),

                   decltype(linear_mp_feedback_controller),
                   // decltype(linear_vel_pid_controller),
                   decltype(linear_vel_slew_controller),
                   decltype(linear_vel_clamp_controller),

                   decltype(angular_vel_pid_controller),
                   decltype(angular_vel_slew_controller),
                   decltype(angular_vel_clamp_controller),

                   // lateral controllers
                   decltype(lateral_controller),
                   decltype(lateral_vel_controller),

                   decltype(linear_voltage_constraints),
                   decltype(angular_voltage_constraints)>
  controllers;

// normal tolerances
extern Tolerances<decltype(linear_tolerances_config.error),
                  decltype(linear_tolerances_config.velocity),
                  decltype(linear_tolerances_config.halfCircle)>
  linearTolerances;

extern Tolerances<decltype(angular_tolerances_config.error),
                  decltype(angular_tolerances_config.velocity)>
  angularTolerances;

// large tolerances
extern Tolerances<decltype(linear_tolerances_config.large_error),
                  decltype(linear_tolerances_config.large_velocity),
                  decltype(linear_tolerances_config.large_halfCircle)>
  largeLinearTolerances;

extern Tolerances<decltype(angular_tolerances_config.large_error),
                  decltype(angular_tolerances_config.large_velocity)>
  largeAngularTolerances;

// chain tolerances
extern Tolerances<decltype(linear_tolerances_config.chain_error),
                  decltype(linear_tolerances_config.chain_halfCircle)>
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

// executors
extern RunExecutor run;
extern AsyncExecutor async;

// extern MotionBuilder<decltype(chassis), decltype(controllers)> mb_blazing;

// extern Chassis<decltype(drivetrain),
//                decltype(vexmaps_tracker),
//                decltype(tolerances)>
//   vexmaps_chassis;

extern Chassis<decltype(drivetrain),
               decltype(vexmaps_tracker),
               decltype(tolerances)>
  vexmaps_chassis;
extern Chassis<decltype(drivetrain), decltype(tracker), decltype(tolerances)>
  blazing_chassis;

// MotionBuilder<decltype(vexmaps_chassis), decltype(controllers)>
//   mb(vexmaps_chassis, controllers);

// extern MotionBuilder<decltype(blazing_chassis), decltype(controllers)> mb;
extern MotionBuilder<decltype(vexmaps_chassis), decltype(controllers)> mb;
extern MotionBuilder<decltype(vexmaps_chassis), decltype(controllers)> mb_vel;

extern ChainedExecutor chain;

// motion things

// custom cos-like func
double angular_linear_func(Angle angle);
