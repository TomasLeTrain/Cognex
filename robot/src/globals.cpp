#include "globals.h"

// pointer to pose tracker
vexmaps::LocalizationModel* pose_getter = &pf_motion_model;
// vexmaps::LocalizationModel* pose_getter = &smoother_model;

pros::Mutex pose_mutex;

// ???
vexmaps::LocalizationModel* orientation_getter = nullptr;

// clang-format off
// motor groups
pros::MotorGroup left_motors({ -11, -14, 13 }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
pros::MotorGroup right_motors({ 15, 16, -10 }, pros::MotorGears::blue, pros::MotorEncoderUnits::rotations);
// clang-format on

// inertial sensor
vexmaps::ScaledIMU imu(1, (360.0 + 3.8) / 360.0);

// intake motors
pros::Motor intake_motor(17);
pros::Motor score_motor(-7);
pros::Motor bin_motor(2);

pros::Optical middle_intake_color_sensor(8);
pros::Optical bottom_intake_color_sensor(3);

// pistons
pros::adi::DigitalOut intake_recycle_piston('A', false);
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

// if the tracker is not installed the list can be left empty -> tracker = {};
std::initializer_list<HorizontalOdometryTracker*> horizontal_trackers = {
    &horizontal_tracker
};
std::initializer_list<VerticalOdometryTracker*> vertical_trackers = {
    &vertical_tracker
};

// blazing tracker
ArcOdomTracker tracker(
  // forward trackers
  { forwards_tracker, left_motor_tracker, right_motor_tracker },
  // sideways trackers
  { sideways_tracker },
  // imus
  { TrackingImu(&imu) });

// custom pf configs - probably can leave alone
vexmaps::MotionModelConfig motion_model_config = {};
// vexmaps::PFConfiguration Pfconfig = {.logging=true,.particle_logging=false};
vexmaps::PFConfiguration Pfconfig = {
    .logging = false,
    .particle_logging = false,
    // .custom_particle_logging=true,
};
vexmaps::SmootherConfig smoother_config = {};

// distance sensor offsets
units::Pose front_distance_offsets = { 7_in, -3.59375_in, 0_stDeg };
units::Pose left_distance_offsets = { 1_in, 4.75_in, 90_stDeg };
units::Pose back_distance_offsets = { -7.125_in, 3.5_in, 180_stDeg };
units::Pose right_distance_offsets = { 1_in, -4.75_in, 270_stDeg };

/* drivetrain / pid configuration */

// NOTE: remember to update every time the drivetrain changes!
drivetrain_config_t drivetrain_config { .track_width = 10.5_in,
                                        .wheel_diameter = 3.25_in,
                                        .rpm = 450_rpm };

// units are in inches
linear_pid_config_t linear_pid_config { .kp = 4.7,
                                        .ki = 0,
                                        .kd = 1,
                                        .windupRange = 7 };

// units are in degrees
angular_pid_config_t angular_pid_config { .kp = 2.8,
                                          .ki = 0,
                                          .kd = 5,
                                          .windupRange = 14 };

LinearSlewController linear_slew(0.2_volt, 0.08_volt);
AngularSlewController angular_slew(0.3_volt);

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
    .chain_error = { 15_stDeg },
};

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
