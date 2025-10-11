#pragma once

#include "apis.h"
#include "auton_globals.h"

// easier use of libs
using namespace blazing;
using namespace vexmaps;

// only variable which cannot be set on globals.cpp
// constexpr size_t pf_particle_count = 10000;
constexpr size_t pf_particle_count = 500;

// structs for helping specify configs
struct drivetrain_config_t {
    Length track_width;
    Length wheel_diameter;
    AngularVelocity rpm;
};

struct linear_pid_config_t {
    double kp;
    double ki;
    double kd;
    std::optional<double> windupRange = std::nullopt;
    std::optional<double> maxVoltage = 127;
    Time timeUnits = 50_msec;
    Length inputUnits = 1_in;
    Voltage outputUnits = Voltage(1.0 / 127.0);
};

struct angular_pid_config_t {
    double kp;
    double ki;
    double kd;
    std::optional<double> windupRange = std::nullopt;
    std::optional<double> maxVoltage = 127;
    Time timeUnits = 50_msec;
    Angle inputUnits = 1_stDeg;
    Voltage outputUnits = Voltage(1.0 / 127.0);
};

// default exits are made to never trigger unless error is set
template<typename T>
struct tolerances_config_t {
    Time duration = 1000_sec;
    ErrorTolerance<T> error = T(0);
    VelocityTolerance<T> velocity = T(100000) / sec;

    Time large_duration = 1000_sec;
    ErrorTolerance<T> large_error = T(0);
    VelocityTolerance<T> large_velocity = T(100000) / sec;

    Time chain_duration = 1000_sec;
    ErrorTolerance<T> chain_error = T(0);
    VelocityTolerance<T> chain_velocity = T(100000) / sec;
};

extern drivetrain_config_t drivetrain_config;
extern linear_pid_config_t linear_pid_config;
extern angular_pid_config_t angular_pid_config;

extern tolerances_config_t<Length> linear_tolerances_config;
extern tolerances_config_t<Angle> angular_tolerances_config;

// different vexmaps configurations
extern vexmaps::MotionModelConfig motion_model_config;
extern vexmaps::PFConfiguration Pfconfig;
extern vexmaps::SmootherConfig smoother_config;

// pointers to be able to change the pose getter
// TODO: replace all that with model manager
extern vexmaps::LocalizationModel* pose_getter;
extern vexmaps::LocalizationModel* orientation_getter;
extern pros::Mutex pose_mutex;

// likely does not need to change
struct CustomDistanceSensorConfiguration {
    // all floats without units are in meters
    static constexpr double exp_l = 1.5;
    static constexpr double std_deviation = (2_in).internal();

    // sum of coefficients 1
    static constexpr double randomCoeff = 0.125;
    static constexpr double expCoeff = 0.075;
    static constexpr double normalCoeff = 0.8;

    static constexpr bool logging = false;
};

/*
 * Hardware configuration
 * add any hardware here and define it in globals.cpp as well
 * */

// controller - can probably leave alone forever
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

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

/*
 * vexmaps configuration
 * */

// trackers
inline vexmaps::MotorGroupTracking
  left_dt_tracker(&left_motors,
                  drivetrain_config.wheel_diameter,
                  drivetrain_config.rpm,
                  -drivetrain_config.track_width / 2);

inline vexmaps::MotorGroupTracking
  right_dt_tracker(&right_motors,
                   drivetrain_config.wheel_diameter,
                   drivetrain_config.rpm,
                   drivetrain_config.track_width / 2);

extern vexmaps::HorizontalOdometryTracker horizontal_tracker;
extern vexmaps::VerticalOdometryTracker vertical_tracker;

struct tracker_config_t {
    Length diameter;
    Length offset;
};

extern tracker_config_t sideways_tracker_config;
extern tracker_config_t forwards_tracker_config;

inline vexmaps::HorizontalOdometryTracker
  horizontal_tracker(&sideways_odom_rotation,
                     sideways_tracker_config.diameter,
                     1,
                     sideways_tracker_config.offset);
inline vexmaps::VerticalOdometryTracker
  vertical_tracker(&forwards_odom_rotation,
                   forwards_tracker_config.diameter,
                   1,
                   forwards_tracker_config.offset);

// lists of intalled trackers
extern std::initializer_list<HorizontalOdometryTracker*> horizontal_trackers;
extern std::initializer_list<VerticalOdometryTracker*> vertical_trackers;

inline vexmaps::PfMotionModel<vexmaps::OdometryModel>
  pf_motion_model(motion_model_config,
                  &left_dt_tracker,
                  &right_dt_tracker,
                  horizontal_trackers,
                  vertical_trackers,
                  &imu,
                  false); // use drivetrain -
                          // can be left on false since it falls back to
                          // drivetrain of no rotations are connected

// distance sensors
extern units::Pose front_distance_offsets;
extern units::Pose left_distance_offsets;
extern units::Pose back_distance_offsets;
extern units::Pose right_distance_offsets;

using laser_model_type =
  vexmaps::DistanceSensorModel<CustomDistanceSensorConfiguration>;

// clang-format off
inline laser_model_type front_laser_model(&front_distance, front_distance_offsets, "front");
inline laser_model_type left_laser_model(&left_distance,   left_distance_offsets,  "left");
inline laser_model_type back_laser_model(&back_distance,   back_distance_offsets,  "back");
inline laser_model_type right_laser_model(&right_distance, right_distance_offsets, "right");
// clang-format on

inline vexmaps::ParticleFilterModel<pf_particle_count>
  pf_model(&pf_motion_model,
           { // distance sensors
             &front_laser_model,
             &left_laser_model,
             &back_laser_model,
             &right_laser_model },
           Pfconfig);

inline vexmaps::SmootherModel
  smoother_model(&pf_motion_model, &pf_model, smoother_config);

/*
 * blazing configs stuff
 *
 */

inline DifferentialDrivetrain drivetrain(&left_motors, &right_motors);

inline ForwardsTracker left_motor_tracker(&left_motors,
                                          -drivetrain_config.track_width / 2,
                                          drivetrain_config.wheel_diameter,
                                          drivetrain_config.rpm);
inline ForwardsTracker right_motor_tracker(&right_motors,
                                           drivetrain_config.track_width / 2,
                                           drivetrain_config.wheel_diameter,
                                           drivetrain_config.rpm);

inline ForwardsTracker forwards_tracker(&sideways_odom_rotation,
                                        forwards_tracker_config.offset,
                                        forwards_tracker_config.diameter);

inline SidewaysTracker sideways_tracker(&forwards_odom_rotation,
                                        sideways_tracker_config.offset,
                                        sideways_tracker_config.diameter);

extern ArcOdomTracker tracker;

// controller stuff
extern PID<Length, Voltage> linear_pid;
extern PID<Angle, Voltage> angular_pid;

extern LinearSlewController linear_slew;
extern AngularSlewController angular_slew;

extern LinearVoltageClampController linear_voltage_constraints;
extern AngularVoltageClampController angular_voltage_constraints;

inline PIDLinearController linear_pid_controller(linear_pid);
inline PIDAngularController angular_pid_controller(angular_pid);

inline Controllers controllers(
  // pid controllers
  linear_pid_controller,
  angular_pid_controller,

  // slew controllers
  linear_slew,
  angular_slew,

  // voltage constraints controllers
  // (included just so they can be set per motion)
  linear_voltage_constraints,
  angular_voltage_constraints);

// normal tolerances
inline Tolerances linearTolerances(linear_tolerances_config.duration,
                                   linear_tolerances_config.error,
                                   linear_tolerances_config.velocity);

inline Tolerances angularTolerances(angular_tolerances_config.duration,
                                    angular_tolerances_config.error,
                                    angular_tolerances_config.velocity);

// large tolerances
inline Tolerances
  largeLinearTolerances(linear_tolerances_config.large_duration,
                        linear_tolerances_config.large_error,
                        linear_tolerances_config.large_velocity);

inline Tolerances
  largeAngularTolerances(angular_tolerances_config.large_duration,
                         angular_tolerances_config.large_error,
                         angular_tolerances_config.large_velocity);

// chain tolerances
inline Tolerances
  chainLinearTolerances(linear_tolerances_config.chain_duration,
                        linear_tolerances_config.chain_error
                        // linear_tolerances_config.chain_velocity
  );

inline Tolerances
  chainAngularTolerances(angular_tolerances_config.chain_duration,
                         angular_tolerances_config.chain_error
                         // angular_tolerances_config.chain_velocity
  );

inline normalLargeChainTolerances tolerances(linearTolerances,
                                             angularTolerances,
                                             largeLinearTolerances,
                                             largeAngularTolerances,

                                             chainLinearTolerances,
                                             chainAngularTolerances);

inline Chassis chassis(drivetrain, tracker, tolerances);

// executors
inline RunExecutor run;
inline AsyncExecutor async;

inline MotionBuilder mb(chassis, controllers);

extern ChainedExecutor chain;

// motion things

// custom cos-like func
double angular_linear_func(Angle angle);
