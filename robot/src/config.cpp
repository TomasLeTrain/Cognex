#include "apis.h"
//

#include "globals.h"
#include <cstddef>

using namespace blazing;
using namespace vexmaps;

// main configurations

// clang-format off
// motor groups
pros::MotorGroup left_motors({ -12, -13, 14 }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
pros::MotorGroup right_motors({ 7, 17, -16 }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
// clang-format on

// inertial sensor
vexmaps::ScaledIMU imu(15, (360.0 + 3.8) / 360.0);

// intake motors
pros::Motor bottom_motor(-19);
pros::Motor top_motor(-1);

pros::Optical middle_intake_color_sensor(8);
pros::Optical bottom_intake_color_sensor(21);

// pistons
pros::adi::DigitalOut intake_stop_piston('H', true);
pros::adi::DigitalOut matchloader_piston('G', false);
pros::adi::DigitalOut wings_piston('F', false);
pros::adi::DigitalOut odom_retract_piston('E', false);

// odom rotation sensors
// pros::Rotation forwards_odom_rotation(-20);
pros::Rotation forwards_odom_rotation(-4);
pros::Rotation sideways_odom_rotation(3);

// particle filter distance sensors
pros::Distance front_distance(6);
pros::Distance back_distance(11);
pros::Distance left_distance(5);
pros::Distance right_distance(10);

// distance sensor offsets
units::Pose front_distance_offsets = { 5.8_in, 4.75_in, 0_stDeg };
// ????
units::Pose left_distance_offsets = { 2.25_in, 5.25_in, 90_stDeg };
units::Pose back_distance_offsets = { -4.4_in, 4.5_in, 180_stDeg };
units::Pose right_distance_offsets = { 2.25_in, -5.25_in, 270_stDeg };

/* vexmaps configuration */

// tracker configs - same signs as lemlib
tracker_config_t forwards_tracker_config = {
    .diameter = 1.9654_in,
    .offset = -0.045_in,
};

tracker_config_t sideways_tracker_config = {
    .diameter = 1.9869_in,
    .offset = 0.45_in,
};

/* drivetrain / pid configuration */

// NOTE: remember to update every time the drivetrain changes!
drivetrain_config_t drivetrain_config { .track_width = 10.5_in,
                                        .wheel_diameter = 3.25_in,
                                        .rpm = 450_rpm };

// units are in inches
linear_pid_config_t linear_pid_config { .kp = 4.5,
                                        .ki = 0,
                                        .kd = 3.6,
                                        .windupRange = 7,
                                        .maxVoltage = 127 };

// units are in degrees
angular_pid_config_t angular_pid_config {
    .kp = 2.5,
    .ki = 0,
    .kd = 4.1,
    .windupRange = 14,
    .maxVoltage = 127,
};

LinearSlewController linear_slew(0.07_volt, 0.06_volt);
AngularSlewController angular_slew(0.8_volt);

LinearSlewController driver_linear_slew(0.1_volt, 0.09_volt);

LinearVoltageClampController linear_voltage_constraints;
AngularVoltageClampController angular_voltage_constraints;

// tolerances
tolerances_config_t<Length> linear_tolerances_config {
    .duration = 200_msec,
    .error { 3_in },
    .velocity { 200_inps },

    .large_duration = 1_sec,
    .large_error { 5_in },
    .large_velocity { 300_inps },

    .chain_duration = 1_sec,
    .chain_error { 6_in },
};

tolerances_config_t<Angle> angular_tolerances_config {
    .duration = 200_msec,
    .error = { 8_stDeg },
    .velocity = { 400_degps },

    .large_duration = 1_sec,
    .large_error = { 15_stDeg },
    .large_velocity = { 300_degps },

    .chain_duration = 1_sec,
    .chain_error = { 20_stDeg },
};

// custom pf configs - probably can leave alone
vexmaps::MotionModelConfig motion_model_config = {};
vexmaps::PFConfiguration Pfconfig = {
    .logging = false,
    .particle_logging = false,
    // .custom_particle_logging=true,
};
vexmaps::SmootherConfig smoother_config = {};

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
DifferentialDrivetrain drivetrain(&left_motors, &right_motors);

ForwardsTracker left_motor_tracker(&left_motors,
                                   -drivetrain_config.track_width / 2,
                                   drivetrain_config.wheel_diameter,
                                   drivetrain_config.rpm);
ForwardsTracker right_motor_tracker(&right_motors,
                                    drivetrain_config.track_width / 2,
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
  { &sideways_tracker },
  // imus
  { &imu_tracker });

// controller stuff
PID<Length, Voltage> linear_pid(linear_pid_config.kp,
                                linear_pid_config.ki,
                                linear_pid_config.kd,
                                linear_pid_config.windupRange,
                                linear_pid_config.maxVoltage,
                                linear_pid_config.timeUnits,
                                linear_pid_config.inputUnits,
                                linear_pid_config.outputUnits);

PID<Angle, Voltage> angular_pid(angular_pid_config.kp,
                                angular_pid_config.ki,
                                angular_pid_config.kd,
                                angular_pid_config.windupRange,
                                angular_pid_config.maxVoltage,
                                angular_pid_config.timeUnits,
                                angular_pid_config.inputUnits,
                                angular_pid_config.outputUnits);

PIDLinearController linear_pid_controller(linear_pid);
PIDAngularController angular_pid_controller(angular_pid);

Controllers<decltype(linear_pid_controller),
            decltype(angular_pid_controller),
            decltype(linear_slew),
            decltype(angular_slew),
            decltype(linear_voltage_constraints),
            decltype(angular_voltage_constraints)>
  controllers(
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
Tolerances<decltype(linear_tolerances_config.error),
           decltype(linear_tolerances_config.velocity)>
  linearTolerances(linear_tolerances_config.duration,
                   linear_tolerances_config.error,
                   linear_tolerances_config.velocity);

Tolerances<decltype(angular_tolerances_config.error),
           decltype(angular_tolerances_config.velocity)>
  angularTolerances(angular_tolerances_config.duration,
                    angular_tolerances_config.error,
                    angular_tolerances_config.velocity);

// large tolerances
Tolerances<decltype(linear_tolerances_config.large_error),
           decltype(linear_tolerances_config.large_velocity)>
  largeLinearTolerances(linear_tolerances_config.large_duration,
                        linear_tolerances_config.large_error,
                        linear_tolerances_config.large_velocity);

Tolerances<decltype(angular_tolerances_config.large_error),
           decltype(angular_tolerances_config.large_velocity)>
  largeAngularTolerances(angular_tolerances_config.large_duration,
                         angular_tolerances_config.large_error,
                         angular_tolerances_config.large_velocity);

// chain tolerances
Tolerances<decltype(linear_tolerances_config.chain_error)>
  chainLinearTolerances(linear_tolerances_config.chain_duration,
                        linear_tolerances_config.chain_error
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

Chassis<decltype(drivetrain), decltype(tracker), decltype(tolerances)>
  chassis(drivetrain, tracker, tolerances);

// executors
RunExecutor run;
AsyncExecutor async;

MotionBuilder<decltype(chassis), decltype(controllers)> mb(chassis,
                                                           controllers);

// same as default chain lerp
auto chain_lerp = [](Voltage a, Voltage b, double t) -> Voltage {
    return (1 - t) * a + t * b;
};

ChainedExecutor chain(100_msec, chain_lerp);

// custom cos-like func
double angular_linear_func(Angle angle) {
    // reduces the domain to [0,pi]
    angle = units::abs(units::constrainAngle180(angle));

    // defined on the range [0,pi/2]
    auto func = [](double x) -> double {
        double poly = 0.0001;
        if (x < 1.224747) {
            // simple polynomial that delays linear output until angle error is
            // small
            poly = 1.0 - 2.0 * (x * x) + 1.08866 * (x * x * x);
        }
        // return 0.00001;
        return 0.7 * poly + std::cos(x) * 0.3;
    };

    // makes this function apply on the range [0,pi]
    if (angle <= rot / 2.0) {
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
vexmaps::ModelManager model_manager(
  {
    { &pf_motion_model, "odom model",     1 },
    { &pf_model,        "pf model",       2 },
    { &smoother_model,  "smoother model", 3 },
},
  &smoother_model);

// if the tracker is not installed the list can be left empty -> tracker = {};
std::initializer_list<HorizontalOdometryTracker*> horizontal_trackers = {
    &horizontal_tracker
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

// never really changes
vexmaps::PfMotionModel<vexmaps::OdometryModel>
  pf_motion_model(motion_model_config,
                  &left_dt_tracker,
                  &right_dt_tracker,
                  horizontal_trackers,
                  vertical_trackers,
                  &imu,
                  false); // use drivetrain -
                          // can be left on false since it falls back to
                          // drivetrain of no rotations are connected

// init map reader
MapReader<> map_reader;

// clang-format off
laser_model_type front_laser_model(&front_distance, front_distance_offsets, "front", &map_reader);
laser_model_type left_laser_model(&left_distance,   left_distance_offsets,  "left",  &map_reader);
laser_model_type back_laser_model(&back_distance,   back_distance_offsets,  "back",  &map_reader);
laser_model_type right_laser_model(&right_distance, right_distance_offsets, "right", &map_reader);
// clang-format on

vexmaps::ParticleFilterModel<pf_particle_count> pf_model(&pf_motion_model,
                                                         { // distance sensors
                                                           &front_laser_model,
                                                           &left_laser_model,
                                                           &back_laser_model,
                                                           &right_laser_model },
                                                         Pfconfig);

vexmaps::SmootherModel
  smoother_model(&pf_motion_model, &pf_model, smoother_config);
