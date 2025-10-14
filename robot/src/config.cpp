#include "globals.h"

using namespace blazing;
using namespace vexmaps;

// main configurations

// clang-format off
// motor groups
pros::MotorGroup left_motors({ -11, -14, 13 }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
pros::MotorGroup right_motors({ 15, 16, -10 }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
// clang-format on

// inertial sensor
vexmaps::ScaledIMU imu(1, (360.0 + 3.8) / 360.0);

// intake motors
pros::Motor bottom_motor(17);
pros::Motor top_motor(2);

pros::Optical middle_intake_color_sensor(8);
pros::Optical bottom_intake_color_sensor(3);

// pistons
pros::adi::DigitalOut intake_raise_piston('B', false);
pros::adi::DigitalOut matchloader_piston('C', false);

// odom rotation sensors
pros::Rotation forwards_odom_rotation(-20);
pros::Rotation sideways_odom_rotation(5);

// particle filter distance sensors
pros::Distance front_distance(9);
pros::Distance back_distance(12);
pros::Distance left_distance(4);
pros::Distance right_distance(19);

// distance sensor offsets
units::Pose front_distance_offsets = { 7_in, -3.59375_in, 0_stDeg };
units::Pose left_distance_offsets = { 1_in, 4.75_in, 90_stDeg };
units::Pose back_distance_offsets = { -7.125_in, 3.5_in, 180_stDeg };
units::Pose right_distance_offsets = { 1_in, -4.75_in, 270_stDeg };

/* vexmaps configuration */

// tracker configs - same signs as lemlib
tracker_config_t forwards_tracker_config = {
    .diameter = 1.995_in,
    .offset = -0.44_in,
};

tracker_config_t sideways_tracker_config = {
    .diameter = 1.96_in,
    .offset = -0.15_in,
};

/* drivetrain / pid configuration */

// NOTE: remember to update every time the drivetrain changes!
drivetrain_config_t drivetrain_config { .track_width = 10.5_in,
                                        .wheel_diameter = 3.25_in,
                                        .rpm = 450_rpm };

// units are in inches
linear_pid_config_t linear_pid_config { .kp = 5,
                                        .ki = 0,
                                        .kd = 1,
                                        .windupRange = 7,
                                        .maxVoltage = 127 };

// units are in degrees
angular_pid_config_t angular_pid_config {
    .kp = 2.8,
    .ki = 0,
    .kd = 6,
    .windupRange = 14,
    .maxVoltage = 127,
};

LinearSlewController linear_slew(std::nullopt, 0.06_volt);
AngularSlewController angular_slew(0.5_volt);

LinearVoltageClampController linear_voltage_constraints;
AngularVoltageClampController angular_voltage_constraints;

// tolerances
tolerances_config_t<Length> linear_tolerances_config {
    .duration = 150_msec,
    .error { 2.5_in },
    .velocity { 20_inps },

    .large_duration = 1_sec,
    .large_error { 5_in },
    .large_velocity { 30_inps },

    .chain_duration = 1_sec,
    .chain_error { 6_in },
};

tolerances_config_t<Angle> angular_tolerances_config {
    .duration = 150_msec,
    .error = { 4_stDeg },
    .velocity = { 20_degps },

    .large_duration = 1_sec,
    .large_error = { 12_stDeg },
    .large_velocity = { 30_degps },

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

// blazing tracker
ArcOdomTracker tracker(
  // forward trackers
  { forwards_tracker, left_motor_tracker, right_motor_tracker },
  // sideways trackers
  { sideways_tracker },
  // imus
  { TrackingImu(&imu) });

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

// clang-format off
laser_model_type front_laser_model(&front_distance, front_distance_offsets, "front");
laser_model_type left_laser_model(&left_distance,   left_distance_offsets,  "left");
laser_model_type back_laser_model(&back_distance,   back_distance_offsets,  "back");
laser_model_type right_laser_model(&right_distance, right_distance_offsets, "right");
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
