#include "api.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "units/units.hpp"

/* add any hardware here and define it in globals.cpp as well */

// controller - can probably leave alone forever
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

// inertial sensor
extern pros::Imu imu;

// intake motor/s?
extern pros::Motor intake_motor;

// pistons
extern pros::adi::DigitalOut matchloader_piston;

// odom rotation sensors
extern pros::Rotation horizontal_odom_rotation;
extern pros::Rotation vertical_odom_rotation;

// particle filter distance sensors
extern pros::Distance front_distance;
extern pros::Distance back_distance ;
extern pros::Distance left_distance ;
extern pros::Distance right_distance;



/* auton related stuff - can be left alone */
enum alliance_t {red = 0, blue = 1};
enum field_side_t {left = 0, right = 1};
enum corner_t {red_left = 0, red_right = 1, blue_left = 2, blue_right = 3};

inline alliance_t alliance = red;
inline field_side_t auto_side = left;
inline corner_t auto_corner = red_left;

/*
 * lemlib config stuff - can likely leave alone forever
 *
*/
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


//
inline lemlib::Drivetrain drivetrain(&left_motor_group,
                              &right_motor_group,
                              drivetrain_config.track_width,
                              drivetrain_config.wheel_diameter,
                              drivetrain_config.rpm,
                              drivetrain_config.horizontal_drift
);

inline lemlib::ControllerSettings lateral_controller(
        lateral_pid_config.P,
                                              lateral_pid_config.I,
                                              lateral_pid_config.D,
                                              lateral_pid_config.anti_windup,
                                              lateral_pid_config.small_error_range,
                                              lateral_pid_config.small_error_range_timeout,
                                              lateral_pid_config.large_error_range,
                                              lateral_pid_config.large_error_range_timeout,
                                              lateral_pid_config.maximum_accel
);

inline lemlib::ControllerSettings angular_controller(angular_pid_config.P,
                                              angular_pid_config.I,
                                              angular_pid_config.D,
                                              angular_pid_config.anti_windup,
                                              angular_pid_config.small_error_range,
                                              angular_pid_config.small_error_range_timeout,
                                              angular_pid_config.large_error_range,
                                              angular_pid_config.large_error_range_timeout,
                                              angular_pid_config.maximum_accel
);

// odometry settings
inline lemlib::OdomSensors sensors(
    nullptr,
    nullptr,
    nullptr,
    nullptr,
    &imu
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
                        // &throttle_curve, 
                        // &steer_curve
);
