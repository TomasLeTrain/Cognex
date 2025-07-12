#include "main.h"
#include "globals.h"
#include "screen/screen.h"

void initialize() {
    // need to start localization tasks and lemlib related things

    // imu calibration
    int attempt = 1;
    bool calibrated = false;
    // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
    while (attempt <= 5) {
        imu.reset();
        // wait until IMU is calibrated
        do pros::delay(10);
        while (sensors.imu->get_status() != pros::ImuStatus::error && sensors.imu->is_calibrating());
        // exit if imu has been calibrated
        if (std::isfinite(sensors.imu->get_rotation())) {
            printf("calibrated!\n");
            calibrated = true;
            break;
        }
        // indicate error
        pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
        printf("IMU failed to calibrate! Attempt #%d\n", attempt);
        attempt++;
    }
    // check if calibration attempts were successful
    if (attempt > 5) { printf("IMU calibration failed, just give up\n"); }

    // pros::delay(200);

    // initialize all models
    pf_motion_model.init();
    pf_model.init();
    smoother_model.init();

    // initialize tasks
    pros::Task odom_task {[&] {
        while (true) {
            uint32_t current_time = pros::millis();
            pf_motion_model.update();
            pros::c::task_delay_until(&current_time, to_msec(pf_motion_model.getTaskDeltaTime()));
        }
    },"odom task"};
    pros::delay(100);

    pros::Task pf_task {[&] {
        while (true) {
            uint32_t current_time = pros::millis();
            pf_model.update();
            pros::c::task_delay_until(&current_time, to_msec(pf_model.getTaskDeltaTime()));
        }
    },"pf task"};
    pros::delay(100);

    pros::Task smoother_task {[&] {
        while (true) {
            uint32_t current_time = pros::millis();
            smoother_model.update();
            pros::c::task_delay_until(&current_time, to_msec(smoother_model.getTaskDeltaTime()));
        }
    },"smoother task"};
    pros::delay(100);

    // constantly updates lemlib's pose
    pros::Task lemlib_pose_task {[&] {
        while (true) {
            uint32_t current_time = pros::millis();

            // should update lemlib pose
            units::Pose curr_pose = pose_getter->getPose();
            Angle curr_orientation = curr_pose.orientation;

            if (orientation_getter != nullptr) {
                Angle other_orientation = orientation_getter->getPose().orientation;
                if (std::isfinite(other_orientation.internal())) {
                    // falls back on pose getter if orientation is infinity
                    curr_orientation = other_orientation;
                }
            }

            // its likely fine to set infinity values since that likely make the program stop its movement as compared
            // to not updating the pose
            chassis.setPose(
                    to_in(curr_pose.x),
                    to_in(curr_pose.y),
                    to_cDeg(curr_orientation),
                    false);

            // performed quickly to get the latest information as soon as possible
            pros::c::task_delay_until(&current_time, 6);
        }
    }};

    // initialize was performed
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

    // initialize screens
    screen::init();
}
