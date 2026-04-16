#include "apis.h"
//

#include "blazing/controllers/controllers.hpp"
#include "blazing/controllers/slew.hpp"
#include "blazing/executor.hpp"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/config.h"
#include "lyfast/drivetrains/velocity_differential.hpp"
#include "pros/abstract_motor.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "vexmaps/mcl/distance_model.hpp"
#include <cmath>
#include <functional>

using namespace blazing;
using namespace vexmaps;

// main configurations

// clang-format off
// motor groups

int8_t left_front = -14;
int8_t left_middle = -13;
int8_t left_back = -12;

int8_t right_front = 17;
int8_t right_middle = 18;
int8_t right_back = 19;

bool vexmaps_logging_enabled = true;
bool custom_particling = true;
// bool custom_particling = false;

pros::MotorGroup left_motors({ left_front, left_middle, left_back }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
pros::MotorGroup right_motors({ right_front, right_middle, right_back }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
// clang-format on

// inertial sensor
// vexmaps::ScaledIMU imu(17, (360.0 + 3.57) / 360.0);
// vexmaps::ScaledIMU imu(11, 361.568120941 / 360.0);
vexmaps::ScaledIMU imu(16, 360.0 / (359.5));
// vexmaps::ScaledIMU imu(15, (360.0 + 1.0) / 360.0);
// vexmaps::ScaledIMU imu(11, 360.0 / 359.0);

// intake motors
// pros::Motor bottom_motor(-19);
// pros::Motor top_motor(-1);

// disable for testing
pros::Motor
  bottom_motor(8, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
pros::Motor
  lever_motor(-2, pros::MotorGears::green, pros::MotorEncoderUnits::rotations);

pros::Optical lower_intake_color_sensor(9);
pros::Optical upper_intake_color_sensor(21);

// pistons
// disable for testing
// pros::adi::DigitalOut intake_stop_piston('H', true);
pros::adi::DigitalOut gate_intake_piston('C', true);
pros::adi::DigitalOut middle_intake_piston('B', true);
pros::adi::DigitalOut middle_intake_piston_2('D', false);
pros::adi::DigitalOut wings_piston('E', false);

pros::adi::DigitalOut matchloader_piston('F', false);
pros::adi::DigitalOut odom_retract_piston('G', false);

pros::adi::DigitalOut bottom_intake_piston('A', false);

// odom rotation sensors
// pros::Rotation forwards_odom_rotation(-20);
pros::Rotation forwards_odom_rotation(15);
pros::Rotation sideways_odom_rotation(21);

// particle filter distance sensors
pros::Distance front_distance(6);
pros::Distance back_distance(5);
pros::Distance left_distance(3);
pros::Distance right_distance(7);

// cor + cor_offsets = geometric
units::V2Position odom_cor_offsets = { 0.0_in, 0_in };

// geometric -> cor
units::V2Position dist_cor_offsets = { 0.0_in, 0_in };

constexpr units::Pose distToCor(units::Pose dist_pose) {
    return { dist_pose - dist_cor_offsets, dist_pose.orientation };
}

// distance sensor offsets

// = 6.3125
auto offset_c = (12.625_in / 2);

units::Pose front_distance_offsets =
  distToCor({ 6.3_in, -offset_c + 3.0_in, 0_stDeg });
units::Pose left_distance_offsets =
  distToCor({ -0.75_in, +offset_c - 2.25_in, 90_stDeg });
units::Pose back_distance_offsets = distToCor({ -4.5_in, 1.5_in, 180_stDeg });
units::Pose right_distance_offsets =
  distToCor({ -0.75_in, -offset_c + 2.25_in, 270_stDeg });

// sunlight: 0.967078567542
// no sunlight: 0.973046024541
double front_distance_scale_factor = 0.973046024541;
// tends to be pretty constant regardless of conditions
Length front_distance_scale_offset = 0.582681261102_in;

// no sunlight: 0.98758896953
double left_distance_scale_factor = 0.98758896953;
Length left_distance_scale_offset = 0.243371397983_in;

// no sunlight: 0.98238
double back_distance_scale_factor = 0.98238;
Length back_distance_scale_offset = 0.0964698_in;

// no sunlight: 0.975042
// double right_distance_scale_factor = 0.975042;
// Length right_distance_scale_offset = 0.395641_in;

double right_distance_scale_factor = 0.980205233729;
Length right_distance_scale_offset = 0.302946477017_in;

/* vexmaps configuration */

// tracker configs - same signs as lemlib
tracker_config_t forwards_tracker_config = {
    .diameter = 1.991_in,
    // geometric is also 0
    .offset = 0.0_in,
};

tracker_config_t sideways_tracker_config = {
    .diameter = 1.991_in,
    // geometric are -2.5, meaning cor is 0.5_in forwards from geometric center
    .offset = -2.6_in,
};

/* drivetrain / pid configuration */

// NOTE: remember to update every time the drivetrain changes!
drivetrain_config_t drivetrain_config { .track_width = 10.3_in,
                                        .track_radius = 10.3_in * 0.5,
                                        .wheel_diameter = 3.25_in,
                                        .rpm = 450_rpm,
                                        .max_velocity = 76_inps,
                                        // (max vel / track_width) * 2
                                        .max_angular_velocity =
                                          (76_inps / 10.5_in) * 2 * Frad,
                                        .input_delay = 40_msec };

// units are in inches
linear_pid_config_t linear_pid_config { .kp = 7.5,
                                        .ki = 0.0,
                                        .kd = 10.3,

                                        .windupRange = 7,
                                        .maxVoltage = 100 };

// units are in degrees
angular_pid_config_t angular_pid_config {
    .kp = 3.15,
    .ki = 0,
    .kd = 5.3,
    .windupRange = 14,
    .maxVoltage = 127,
};

angular_pid_config_t turn_heading_pid_config {
    .kp = 3.50,
    .ki = 0.06,
    .kd = 6.3,
    .windupRange = 45,
    .maxVoltage = 127,
};

angular_pid_config_t matchloader_angular_pid_config {
    // after ki ones - aggressive
    .kp = 3.50, .ki = 0.17, .kd = 6.0, .windupRange = 45, .maxVoltage = 127,
};

LinearSlewController linear_slew { std::nullopt, 0.2_volt };
AngularSlewController angular_slew {};

LinearVoltageClampController linear_voltage_constraints;
AngularVoltageClampController angular_voltage_constraints;

// tolerances
tolerances_config_t<Length> linear_tolerances_config {
    .duration = 100_msec,
    .error { 0.7_in },
    // .error { 1.0_in },
    .velocity { 400_inps },

    .large_duration = 400_sec,
    .large_error { 3_in },
    .large_velocity { 30_inps },

    .chain_duration = 1_sec,
    .chain_error { 3_in },
};

tolerances_config_t<Angle> angular_tolerances_config {
    // good for voltage turns
    // .duration = 200_msec,
    // .error = { 8_stDeg },
    // .velocity = { 400_degps },
    //
    // .large_duration = 1_sec,
    // .large_error = { 15_stDeg },
    // .large_velocity = { 300_degps },
    //
    // .chain_duration = 1_sec,
    // .chain_error = { 20_stDeg },

    // good for tuning
    // .duration = 30_msec,
    // .error = { 1_stDeg },
    // .velocity = { 20_degps },

    .duration = 40_msec,
    .error = { 2.25_stDeg },
    // .velocity = { 60_degps },
    .velocity = { 120_degps },

    // .duration = 30_msec,
    // .error = { 1.2_stDeg },
    // .velocity = { 20_degps },
    //
    .large_duration = 400_msec,
    .large_error = { 7_stDeg },
    .large_velocity = { 70_degps },

    .chain_duration = 1_sec,
    .chain_error = { 20_stDeg },
};

// custom pf configs - probably can leave alone
vexmaps::MotionModelConfig motion_model_config = {};
vexmaps::PFConfiguration Pfconfig = {
    // .logging = false,
    // .particle_logging = false,

    .logging = vexmaps_logging_enabled,
    .particle_logging = vexmaps_logging_enabled,
    //
    .custom_particle_logging = custom_particling && vexmaps_logging_enabled,
    .print_custom_data = vexmaps_logging_enabled,

    .weightPredictionFactor = 0.9,
};

vexmaps::SmootherConfig smoother_config = {
    // for all parameters:
    // 0 = all model
    // 1 = all measurement

    // // determines how much a pose measurement influences the pose estimate
    // good ?
    // .alpha_x = 0.06,
    // .alpha_y = 0.06,
    // .alpha_theta = 0.00,

    // good without cook
    // .alpha_x = 0.04,
    // .alpha_y = 0.04,
    // .alpha_theta = 0.00,

    .alpha_x = 0.16,
    .alpha_y = 0.16,
    .alpha_theta = 0.00,

    // possible good values
    .ang_vel_alpha = 0.10 / 300_degps,
    .theta_to_alpha = 0.07,
    .linear_vel_alpha = 0.00 / 70_inps,
    //
    // .ang_vel_alpha = 0.0 / 300_degps,
    // .theta_to_alpha = 0.0,
    // .linear_vel_alpha = 0.0 / 70_inps,

    // used by pose_delta_measurement to estimate the pose
    .beta_x = 1,
    .beta_y = 1,
    .beta_theta = 1
};

// likely does not need to change
vexmaps::DistanceSensorConfig distance_sensor_config {
    // all floats without units are in meters
    .exp_l = 1.5,
    .std_deviation = (2_in).internal(),
    .map_deviation = (3_in).internal(),

    // sum of coefficients 1
    .randomCoeff = 0.0,
    .expCoeff = 0.15,
    .normalCoeff = 0.6,
    .mapCoeff = 0.25,

    .maxDistanceDifference = 5_in,
    .maxOutDistanceDifference = 5_in,

    .maxUsableDistance = 70_in,

    .detect_obstacles = true,

    .logging = vexmaps_logging_enabled
};

pros::Controller controller(pros::E_CONTROLLER_MASTER);

//
//
//
//
//
//
//
//
// blazing stuff - can keep alone
ForwardsTracker left_motor_tracker(&left_motors,
                                   -drivetrain_config.track_radius,
                                   drivetrain_config.wheel_diameter,
                                   drivetrain_config.rpm);
ForwardsTracker right_motor_tracker(&right_motors,
                                    drivetrain_config.track_radius,
                                    drivetrain_config.wheel_diameter,
                                    drivetrain_config.rpm);

ForwardsTracker forwards_tracker(&forwards_odom_rotation,
                                 forwards_tracker_config.offset,
                                 forwards_tracker_config.diameter);

SidewaysTracker sideways_tracker(&sideways_odom_rotation,
                                 sideways_tracker_config.offset,
                                 sideways_tracker_config.diameter);

TrackingImu imu_tracker(&imu);

// blazing tracker
ArcOdomTracker tracker(
  // forward trackers
  { &forwards_tracker, &left_motor_tracker, &right_motor_tracker },
  // sideways trackers
  // { &sideways_tracker },
  {},
  // imus
  { &imu_tracker },
  odom_cor_offsets);

// voltage controller stuff
PID<Length, Voltage> linear_pid(linear_pid_config.kp,
                                linear_pid_config.ki,
                                linear_pid_config.kd,
                                linear_pid_config.windupRange,
                                linear_pid_config.maxVoltage,
                                linear_pid_config.derivative_alpha,
                                linear_pid_config.timeUnits,
                                linear_pid_config.inputUnits,
                                linear_pid_config.outputUnits);
PID<Angle, Voltage> turn_drive_pid(angular_pid_config.kp,
                                   angular_pid_config.ki,
                                   angular_pid_config.kd,
                                   angular_pid_config.windupRange,
                                   angular_pid_config.maxVoltage,
                                   angular_pid_config.derivative_alpha,
                                   angular_pid_config.timeUnits,
                                   angular_pid_config.inputUnits,
                                   angular_pid_config.outputUnits);
PID<Angle, Voltage> turn_heading_pid(turn_heading_pid_config.kp,
                                     turn_heading_pid_config.ki,
                                     turn_heading_pid_config.kd,
                                     turn_heading_pid_config.windupRange,
                                     turn_heading_pid_config.maxVoltage,
                                     turn_heading_pid_config.derivative_alpha,
                                     turn_heading_pid_config.timeUnits,
                                     turn_heading_pid_config.inputUnits,
                                     turn_heading_pid_config.outputUnits);
PIDLinearController linear_pid_controller(linear_pid);
PIDAngularController angular_pid_controller(turn_drive_pid);
// random not used stuff

lyfast::DifferentialVelocityControllerParams vel_controller_params {
	.linear = {
		// .left_Kv = 0.46 * volt / mps,
		.left_Kv = 0.410935 * volt / mps,
		.left_Ka = 0.09 * volt / mps2,
		.left_Ks = 0.059023 * volt, // subtract to allow settling?
		.left_low_target_Kv = 0.37 * volt / mps,
		.left_low_target_Ka = 0.04 * volt / mps2,
		.left_low_target_Ks = (0.059023 - 0.02) * volt, // subtract to allow settling
		//
		.right_Kv = 0.411732 * volt / mps,
		.right_Ka = 0.09 * volt / mps2,
		.right_Ks = 0.0639167 * volt,
		.right_low_target_Kv = 0.37 * volt / mps,
		.right_low_target_Ka = 0.04 * volt / mps2,
		.right_low_target_Ks = (0.0639167 - 0.02) * volt, // subtract to allow settling
		.Ka_delta_time = 20_msec,
		.low_target_vel_threshold = 20_inps,
		// TODO: tune
		.low_target_accel_threshold = 2000_inps2
	},
	.angular = {
		// closer to 0.49 if using linear ks
		// .left_Kv = 0.50 * volt / mps,
		.left_Kv = 0.67 * volt / mps,
		// .left_Ka = 0.11 * volt / mps2,
		.left_Ka = 0.09 * volt / mps2,
		.left_Ks = 0.08 * volt,

		.left_low_target_Kv = 0.5 * volt / mps,
		.left_low_target_Ka = 0.03 * volt / mps2,
		.left_low_target_Ks = (0.08 - 0.02) * volt,

		// closer to 0.495 if using linear ks
		// .right_Kv = 0.50 * volt / mps,
		.right_Kv = 0.67 * volt / mps,
		// .right_Ka = 0.11 * volt / mps2,
		.right_Ka = 0.09 * volt / mps2,
		.right_Ks = 0.0984043 * volt,

		.right_low_target_Kv = 0.5 * volt / mps,
		.right_low_target_Ka = 0.03 * volt / mps2,
		.right_low_target_Ks = (0.08 - 0.02) * volt,

		// .left_Kv = 0.855 * volt / mps,
		// // .left_Ka = 0.11 * volt / mps2,
		// .left_Ka = 0.07 * volt / mps2,
		// .left_Ks = 0.08 * volt,
		// .left_low_target_Kv = 0.5 * volt / mps,
		// .left_low_target_Ka = 0.0 * volt / mps2,
		// .left_low_target_Ks = 0.08 * volt,
		//
		// .right_Kv = 0.90 * volt / mps,
		// // .right_Ka = 0.11 * volt / mps2,
		// .right_Ka = 0.07 * volt / mps2,
		// .right_Ks = 0.08 * volt,
		// .right_low_target_Kv = 0.5 * volt / mps,
		// .right_low_target_Ka = 0.0 * volt / mps2,
		// .right_low_target_Ks = 0.08 * volt,

		.Ka_delta_time = 20_msec,
		// TODO: tune
		.low_target_vel_threshold = 2_inps,
		// TODO: tune
		// .low_target_accel_threshold = 80_inps2
		.low_target_accel_threshold = 0_inps2,
	},
	.linear_pid = {
		.left_Kp = 0.5 * volt / mps,
		.left_Kp_close = 0.0 * volt / mps,
		.left_Kp_low = 0.0 * volt / mps,
		.left_low_threshold = 7_inps,
		.left_close_threshold = 0_inps,
		// .left_Ki = 1.0 * volt / m,
		.left_Ki = 0.0 * volt / m,
		.left_Ki_windup = 12_inps,
		//
		.left_max_output =  1_volt,
		.left_tbh_factor =  1.0,

		.right_Kp = 0.5 * volt / mps,
		.right_Kp_close = 0.0 * volt / mps,
		.right_Kp_low = 0.0 * volt / mps,
		.right_low_threshold = 7_inps,
		.right_close_threshold = 0_inps,
		// .right_Ki = 1.0 * volt / m,
		.right_Ki = 0.0 * volt / m,
		.right_Ki_windup = 12_inps,

		.right_max_output =  1_volt,
		.right_tbh_factor =  1.0,


		// .left_Kp = 1.5 * volt / mps,
		// .left_Kp_close = 0.0 * volt / mps,
		// .left_Kp_low = 0.0 * volt / mps,
		// .left_low_threshold = 7_inps,
		// .left_close_threshold = 0_inps,
		// // .left_Ki = 1.0 * volt / m,
		// .left_Ki = 0.0 * volt / m,
		// .left_Ki_windup = 12_inps,
		// //
		// .left_max_output =  1_volt,
		// .left_tbh_factor =  1.0,
		//
		// .right_Kp = 1.5 * volt / mps,
		// .right_Kp_close = 0.0 * volt / mps,
		// .right_Kp_low = 0.0 * volt / mps,
		// .right_low_threshold = 7_inps,
		// .right_close_threshold = 0_inps,
		// // .right_Ki = 1.0 * volt / m,
		// .right_Ki = 0.0 * volt / m,
		// .right_Ki_windup = 12_inps,
		//
		// .right_max_output =  1_volt,
		// .right_tbh_factor =  1.0,
	},
	.angular_pid = {
		.left_Kp = 0.5 * volt / mps,
		.left_Kp_close = 0.0 * volt / mps,
		.left_Kp_low = 0.0 * volt / mps,
		.left_low_threshold = 10_inps,
		.left_close_threshold = 0_inps,
		// .left_Ki = 1.5 * volt / m,
		.left_Ki = 0.0 * volt / m,
		.left_Ki_windup = 12_inps,
		//
		.left_max_output =  1_volt,
		.left_tbh_factor =  1.0,

		.right_Kp = 0.5 * volt / mps,
		.right_Kp_close = 0.0 * volt / mps,
		.right_Kp_low = 0.0 * volt / mps,
		.right_low_threshold = 10_inps,
		.right_close_threshold = 0_inps,
		// .right_Ki = 1.5 * volt / m,
		.right_Ki = 0.0 * volt / m,
		.right_Ki_windup = 12_inps,

		.right_max_output =  1_volt,
		.right_tbh_factor =  1.0,
	}
};
lyfast::DifferentialVelocityController vel_controller {
    vel_controller_params,
    76_inps,
    drivetrain_config.track_width,
    false
};

// mp feedback
lyfast::mpFeedback<Length>
  linear_mp_feedback(70_inps, 150_inps2, 0.3_in, 0.05_inps / 0.20_in);

LinearVelocityFeedbackController<decltype(linear_mp_feedback)>
  linear_mp_feedback_controller(linear_mp_feedback);

LinearVelocitySlewController linear_vel_slew_controller { 170_inps2 };
LinearVelocityClampController linear_vel_clamp_controller {};

// end linear velocity stuff //
//
// used for seeking motions
PID<Angle, AngularVelocity> linear_angular_vel_pid(
  14.50,
  0.0,
  10.0,
  to_stRad(10_stDeg), // windup range
  to_radps(drivetrain_config.max_angular_velocity), // restrict max vel
  std::nullopt, // derivative alpha
  50_msec,
  1_stRad,
  1_radps);

PID<Angle, AngularVelocity>
  turn_heading_vel_pid(11.200,
                       0.0,
                       4.900,
                       to_stRad(10_stDeg),
                       to_radps(drivetrain_config.max_angular_velocity),
                       std::nullopt, // derivative alpha
                       50_msec,
                       1_stRad,
                       1_radps);

// start angular velocity stuff
// PIDAngularVelocityController
// angular_vel_pid_controller(linear_angular_vel_pid);
PIDAngularVelocityController angular_vel_pid_controller(linear_angular_vel_pid);

AngularVelocitySlewController angular_vel_slew_controller {};
AngularVelocityClampController angular_vel_clamp_controller {};

// end angular velocity stuff //

// path following stuff //

std::array<float, 3> Q { (30_in).internal(),
                         // (6_in).internal(),
                         // (5_in).internal(),
                         (8_in).internal(),
                         // (1_in).internal(),
                         // (5_stDeg).internal() };
                         (180_stDeg).internal() };

// [x, theta]
std::array<float, 2> simple_Q { (10000_in).internal(),
                                (50000_stDeg).internal() };

std::array<float, 2> simple_R { (1_inps).internal(),
                                // max angular velocity
                                (1_degps).internal() };

std::array<float, 2> R { // max velocity
                         drivetrain_config.max_velocity.internal(),
                         // max angular velocity
                         drivetrain_config.max_angular_velocity.internal()
};

LinearVelocity lqr_minimum_velocity = 1.0_inps;

blazing::lyfast::state_space::LTVUnicycleController
  lqr_controller(Q, R, simple_Q, simple_R, 0_msec, lqr_minimum_velocity);

lyfast::PathPoseFeedbackController<decltype(lqr_controller)>
  path_pose_feedback_controller(lqr_controller);
// end path following stuff //

GlobalControllersT controllers(
  // pid controllers
  linear_pid_controller,
  angular_pid_controller,

  // slew controllers
  linear_slew,
  angular_slew,

  path_pose_feedback_controller,
  // linear velocity controllers
  // linear_vel_pid_controller,
  linear_mp_feedback_controller,
  linear_vel_slew_controller,
  linear_vel_clamp_controller,

  // angular velocity controllers
  angular_vel_pid_controller,
  angular_vel_slew_controller,
  angular_vel_clamp_controller,

  // voltage constraints controllers
  // (included just so they can be set per motion)
  linear_voltage_constraints,
  angular_voltage_constraints);

// normal tolerances
Tolerances<decltype(linear_tolerances_config.error),
           decltype(linear_tolerances_config.velocity),
           decltype(linear_tolerances_config.halfCircle)>
  linearTolerances(linear_tolerances_config.duration,
                   linear_tolerances_config.error,
                   linear_tolerances_config.velocity,
                   linear_tolerances_config.halfCircle);

Tolerances<decltype(angular_tolerances_config.error),
           decltype(angular_tolerances_config.velocity)>
  angularTolerances(angular_tolerances_config.duration,
                    angular_tolerances_config.error,
                    angular_tolerances_config.velocity);

// large tolerances
Tolerances<decltype(linear_tolerances_config.large_error),
           decltype(linear_tolerances_config.large_velocity),
           decltype(linear_tolerances_config.large_halfCircle)>
  largeLinearTolerances(linear_tolerances_config.large_duration,
                        linear_tolerances_config.large_error,
                        linear_tolerances_config.large_velocity,
                        linear_tolerances_config.large_halfCircle);

Tolerances<decltype(angular_tolerances_config.large_error),
           decltype(angular_tolerances_config.large_velocity)>
  largeAngularTolerances(angular_tolerances_config.large_duration,
                         angular_tolerances_config.large_error,
                         angular_tolerances_config.large_velocity);

// chain tolerances
Tolerances<decltype(linear_tolerances_config.chain_error)
           // , decltype(linear_tolerances_config.chain_halfCircle)
           >
  chainLinearTolerances(linear_tolerances_config.chain_duration,
                        linear_tolerances_config.chain_error
                        // linear_tolerances_config.chain_halfCircle
                        // linear_tolerances_config.chain_velocity
  );

Tolerances<decltype(angular_tolerances_config.chain_error)>
  chainAngularTolerances(angular_tolerances_config.chain_duration,
                         angular_tolerances_config.chain_error
                         // angular_tolerances_config.chain_velocity
  );

normalLargeChainTolerances<decltype(linearTolerances),
                           decltype(angularTolerances),
                           decltype(largeLinearTolerances),
                           decltype(largeAngularTolerances),
                           decltype(chainLinearTolerances),
                           decltype(chainAngularTolerances)>
  tolerances(linearTolerances,
             angularTolerances,
             largeLinearTolerances,
             largeAngularTolerances,

             chainLinearTolerances,
             chainAngularTolerances);

// executors
RunExecutor run;
AsyncExecutor async;

// same as default chain lerp
// auto chain_lerp = [](Voltage a, Voltage b, double t) -> Voltage {
//     return (1 - t) * a + t * b;
// };

// ChainedExecutor chain(100_msec, chain_lerp);

// avoids a division by zero
AsyncExecutor chain([](blazing::motionExecutionResult result,
                       AsyncExecutor* executor) {
    return (executor->numQueuedMotions() > 1) &&
           result.inChainTolerance.value_or(false);
});

// custom cos-like func
double angular_linear_func(Angle angle) {
    // reduces the domain to [0,pi]
    angle = units::abs(units::constrainAngle180(angle));

    // double sgn = units::sgn(angle);
    // angle = units::abs(angle);

    // defined on the range [0,pi/2]
    // auto func = [](double x) -> double {
    //     double poly = 0.0001;
    //     if (x < 1.224747) {
    //         // simple polynomial that delays linear output until angle error
    //         is small poly = 1.0 - 2.0 * (x * x) + 1.08866 * (x * x * x);
    //     }
    //     // return 0.00001;
    //     return 0.7 * poly + std::cos(x) * 0.3;
    // };

    // defined on the range [0,pi/2]
    // auto func = [](double x) -> double {
    //     return std::exp(-1.25 * x);
    // };
    //
    // defined on the range [0,pi/2]
    auto func = [](double x) -> double {
        double a = 0.93;
        return std::exp(-a * x) * (1 - (2 / M_PI) * x);
    };

    // makes this function apply on the range [0,pi]
    if (angle <= rot / 4.0) {
        return func(angle.internal());
    } else {
        return -func(M_PI - angle.internal());
    }
};

//
//
//
//
//
//
//
//
// vexmaps configs

vexmaps::HorizontalOdometryTracker
  horizontal_tracker(&sideways_odom_rotation,
                     sideways_tracker_config.diameter,
                     1,
                     sideways_tracker_config.offset);
vexmaps::VerticalOdometryTracker
  vertical_tracker(&forwards_odom_rotation,
                   forwards_tracker_config.diameter,
                   1,
                   forwards_tracker_config.offset);

// if the tracker is not installed the list can be left empty -> tracker = {};
std::initializer_list<HorizontalOdometryTracker*> horizontal_trackers = {
    // &horizontal_tracker
};
std::initializer_list<VerticalOdometryTracker*> vertical_trackers = {
    &vertical_tracker
};

// trackers
vexmaps::MotorGroupTracking left_dt_tracker(&left_motors,
                                            drivetrain_config.wheel_diameter,
                                            drivetrain_config.rpm,
                                            -drivetrain_config.track_width / 2);

vexmaps::MotorGroupTracking right_dt_tracker(&right_motors,
                                             drivetrain_config.wheel_diameter,
                                             drivetrain_config.rpm,
                                             drivetrain_config.track_width / 2);

// never really changes
vexmaps::PfMotionModel<vexmaps::OdometryModel>
  pf_motion_model(motion_model_config,
                  &left_dt_tracker,
                  &right_dt_tracker,
                  horizontal_trackers,
                  vertical_trackers,
                  &imu,
                  odom_cor_offsets,
                  false,
                  // use drivetrain -
                  // can be left on false since it falls back to
                  // drivetrain of no rotations are connected

                  false);

DistanceSensorModel front_laser_model(&front_distance,
                                      front_distance_offsets,
                                      front_distance_scale_factor,
                                      front_distance_scale_offset,
                                      "front",
                                      distance_sensor_config);

DistanceSensorModel left_laser_model(&left_distance,
                                     left_distance_offsets,
                                     left_distance_scale_factor,
                                     left_distance_scale_offset,
                                     "left",
                                     distance_sensor_config);
DistanceSensorModel back_laser_model(&back_distance,
                                     back_distance_offsets,
                                     back_distance_scale_factor,
                                     back_distance_scale_offset,
                                     "back",
                                     distance_sensor_config);
DistanceSensorModel right_laser_model(&right_distance,
                                      right_distance_offsets,
                                      right_distance_scale_factor,
                                      right_distance_scale_offset,
                                      "right",
                                      distance_sensor_config);

vexmaps::ParticleFilterModel<pf_particle_count> pf_model(&pf_motion_model,
                                                         { // distance sensors
                                                           &front_laser_model,
                                                           &left_laser_model,
                                                           &back_laser_model,
                                                           &right_laser_model },
                                                         Pfconfig);

vexmaps::SmootherModel
  smoother_model(&pf_motion_model, &pf_model, smoother_config);

vexmaps::ModelManager model_manager(
  {
    { &pf_motion_model, "odom model",     1 },
    { &pf_model,        "pf model",       2 },
    { &smoother_model,  "smoother model", 3 },
},
  &smoother_model);

BlazingWrapper vexmaps_tracker(&model_manager);
// vexmaps stuff end

lyfast::DrivetrainVelocityPlant drivetrain_plant {
    nullptr,
    nullptr,
    vel_controller,
    drivetrain_config.wheel_diameter
};

// nothing before this point makes references to the drivetrain
lyfast::VelocityDifferentialDrivetrain
  drivetrain(&left_motors,
             &right_motors,
             &drivetrain_plant,
             drivetrain_config.track_width);

Chassis<decltype(drivetrain), decltype(vexmaps_tracker), decltype(tolerances)>
  vexmaps_chassis(&drivetrain, &vexmaps_tracker, tolerances);

Chassis<decltype(drivetrain), decltype(tracker), decltype(tolerances)>
  blazing_chassis(&drivetrain, &tracker, tolerances);

MotionBuilder<decltype(vexmaps_chassis), decltype(controllers)>
  mb(vexmaps_chassis, controllers);

// MotionBuilder<decltype(vexmaps_chassis), decltype(controllers)>
//   mb_vel(vexmaps_chassis, controllers);
// MotionBuilder<decltype(blazing_chassis), decltype(controllers)>
//   mb(blazing_chassis, controllers);
