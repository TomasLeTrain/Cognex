#pragma once

#include "api.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/optical.hpp"
#include "units/units.hpp"
#include "vexmaps/api.hpp"
#include "vexmaps/localization_model.hpp"
#include "vexmaps/mcl/pf_motion_model.hpp"
#include "vexmaps/odometry/odometry.hpp"
#include "vexmaps/odometry/tracking_wheel.hpp"
#include <string>


// only variable which cannot be set on globals.cpp
// constexpr size_t pf_particle_count = 10000;
constexpr size_t pf_particle_count = 500;

// some stuff which is required up here
struct drivetrain_config_t {
        float track_width;
        float wheel_diameter;
        float rpm;
        float horizontal_drift;
};

struct lateral_pid_config_t {
        float P;
        float I;
        float D;
        float anti_windup;
        float small_error_range;
        float small_error_range_timeout;
        float large_error_range;
        float large_error_range_timeout;
        float maximum_accel;
};

struct angular_pid_config_t {
        float P;
        float I;
        float D;
        float anti_windup;
        float small_error_range;
        float small_error_range_timeout;
        float large_error_range;
        float large_error_range_timeout;
        float maximum_accel;
};

extern drivetrain_config_t drivetrain_config;
extern lateral_pid_config_t lateral_pid_config;
extern angular_pid_config_t angular_pid_config;

using horizontalTrackers =
  std::initializer_list<vexmaps::HorizontalOdometryTracker*>;
using verticalTrackers =
  std::initializer_list<vexmaps::VerticalOdometryTracker*>;

// different vexmaps configurations
extern vexmaps::MotionModelConfig motion_model_config;
extern vexmaps::PFConfiguration Pfconfig;
extern vexmaps::SmootherConfig smoother_config;


extern vexmaps::LocalizationModel* pose_getter;
extern vexmaps::LocalizationModel* orientation_getter;

// likely does not need to change
struct CustomDistanceSensorConfiguration {
    // all floats without units are in meters
    static constexpr double exp_l = 1.5;
    static constexpr double std_deviation = (2_in).internal();

    // all these should add to one
    static constexpr double randomCoeff = 0.15 - 0.025;
    static constexpr double expCoeff = 0.1 - 0.025;
    static constexpr double normalCoeff = 0.75 + 0.025 + 0.025;

    static constexpr bool logging = true;
};


/*
 * Hardware configuration
 * add any hardware here and define it in globals.cpp as well
 * */

// controller - can probably leave alone forever
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

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
extern pros::Rotation horizontal_odom_rotation;
extern pros::Rotation vertical_odom_rotation;

// particle filter distance sensors
extern pros::Distance front_distance;
extern pros::Distance back_distance;
extern pros::Distance left_distance;
extern pros::Distance right_distance;

/*
 * vexmaps configuration
 * */

// trackers
inline vexmaps::MotorGroupTracking left_dt_tracker(&left_motor_group, drivetrain_config.wheel_diameter,
                                                      drivetrain_config.rpm, drivetrain_config.track_width / 2);
inline vexmaps::MotorGroupTracking right_dt_tracker(&right_motor_group, drivetrain_config.wheel_diameter,
                                                       drivetrain_config.rpm, -drivetrain_config.track_width / 2);

extern vexmaps::HorizontalOdometryTracker horizontal_tracker;
extern vexmaps::VerticalOdometryTracker vertical_tracker;

extern units::V2Position front_distance_offsets;
extern units::V2Position left_distance_offsets;
extern units::V2Position back_distance_offsets;
extern units::V2Position right_distance_offsets;

// lists of intalled trackers
extern horizontalTrackers horizontal_trackers;
extern verticalTrackers vertical_trackers;

extern Length horizontal_offset;
extern Length vertical_offset;


extern Length hor_odom_wheel_diameter;
extern Length ver_odom_wheel_diameter;

inline vexmaps::HorizontalOdometryTracker horizontal_tracker(&horizontal_odom_rotation, hor_odom_wheel_diameter,1,horizontal_offset);
inline vexmaps::VerticalOdometryTracker vertical_tracker(&vertical_odom_rotation, ver_odom_wheel_diameter, 1, vertical_offset);

inline vexmaps::PfMotionModel<vexmaps::OdometryModel>
  pf_motion_model(motion_model_config,
                  &left_dt_tracker,
                  &right_dt_tracker,
                  horizontal_trackers,
                  vertical_trackers,
                  &imu,
                  false); // use drivetrain -
                          // can be left on false since it falls back to drivetrain of no rotations are connected


// distance sensors
inline vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  front_laser_model(&front_distance, units::Pose(front_distance_offsets, 0_stDeg), "front");
inline vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  left_laser_model(&left_distance, units::Pose(left_distance_offsets, 87.5_stDeg), "left");
inline vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  back_laser_model(&back_distance, units::Pose(back_distance_offsets, 180_stDeg), "back");
inline vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>
  right_laser_model(&right_distance, units::Pose(right_distance_offsets, 271_stDeg), "right");

inline vexmaps::ParticleFilterModel<pf_particle_count> pf_model(
                                                      &pf_motion_model,
                                                      { // distance sensors
                                                          &front_laser_model,
                                                          &left_laser_model,
                                                          &back_laser_model,
                                                          &right_laser_model
                                                      },
                                                      Pfconfig);

inline vexmaps::SmootherModel
  smoother_model(&pf_motion_model, &pf_model, smoother_config);


/*
 * lemlib config stuff - can likely leave alone forever
 *
 */
inline lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, drivetrain_config.track_width,
                                     drivetrain_config.wheel_diameter, drivetrain_config.rpm,
                                     drivetrain_config.horizontal_drift);

inline lemlib::ControllerSettings
    lateral_controller(lateral_pid_config.P, lateral_pid_config.I, lateral_pid_config.D, lateral_pid_config.anti_windup,
                       lateral_pid_config.small_error_range, lateral_pid_config.small_error_range_timeout,
                       lateral_pid_config.large_error_range, lateral_pid_config.large_error_range_timeout,
                       lateral_pid_config.maximum_accel);

inline lemlib::ControllerSettings
    angular_controller(angular_pid_config.P, angular_pid_config.I, angular_pid_config.D, angular_pid_config.anti_windup,
                       angular_pid_config.small_error_range, angular_pid_config.small_error_range_timeout,
                       angular_pid_config.large_error_range, angular_pid_config.large_error_range_timeout,
                       angular_pid_config.maximum_accel);

// odometry settings
inline lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, // drivetrain settings
                               lateral_controller, // lateral PID settings
                               angular_controller, // angular PID settings
                               sensors // odometry sensors
                               // &throttle_curve,
                               // &steer_curve
);

/* auton related stuff - can be left alone */
enum class alliance_t { unset = -1, red = 0, blue = 1 };

enum class field_side_t { unset = -1, left = 0, right = 1 };

// enum class corner_t { unset = -1,  red_left = 0, red_right = 1, blue_left = 2, blue_right = 3 };

inline alliance_t auto_alliance = alliance_t::unset;
// inline alliance_t auto_alliance = alliance_t::red;
inline field_side_t auto_side = field_side_t::unset;
// inline field_side_t auto_side = field_side_t::left;
// inline corner_t auto_corner = corner_t::unset;
inline std::string selected_auton = "";
