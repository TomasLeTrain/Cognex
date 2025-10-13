#include "globals.h"

// blazing tracker
ArcOdomTracker tracker(
  // forward trackers
  { forwards_tracker, left_motor_tracker, right_motor_tracker },
  // sideways trackers
  { sideways_tracker },
  // imus
  { TrackingImu(&imu) });

DifferentialDrivetrain drivetrain(&left_motors, &right_motors);

ForwardsTracker left_motor_tracker(&left_motors,
                                   -drivetrain_config.track_width / 2,
                                   drivetrain_config.wheel_diameter,
                                   drivetrain_config.rpm);
ForwardsTracker right_motor_tracker(&right_motors,
                                    drivetrain_config.track_width / 2,
                                    drivetrain_config.wheel_diameter,
                                    drivetrain_config.rpm);

ForwardsTracker forwards_tracker(&sideways_odom_rotation,
                                 forwards_tracker_config.offset,
                                 forwards_tracker_config.diameter);

SidewaysTracker sideways_tracker(&forwards_odom_rotation,
                                 sideways_tracker_config.offset,
                                 sideways_tracker_config.diameter);

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
