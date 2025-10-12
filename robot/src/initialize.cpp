#include "globals.h"
#include "main.h"
#include "screen/screen.h"
#include <mutex>

void initialize() {
    // need to start localization tasks and lemlib related things

    // imu calibration
    int attempt = 1;
    bool calibrated = false;
    // calibrate inertial, and if calibration fails, then repeat 5 times or
    // until successful
    while (attempt <= 5) {
        imu.reset();
        // wait until IMU is calibrated
        do pros::delay(10);
        while (imu.get_status() != pros::ImuStatus::error &&
               imu.is_calibrating());
        // exit if imu has been calibrated
        if (std::isfinite(imu.get_rotation())) {
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
    if (attempt > 5) {
        printf("IMU calibration failed, just give up\n");
    }

    pros::delay(100);

    // initialize all models
    pf_motion_model.init();
    pf_model.init();
    smoother_model.init();

    // initialize tasks
    pros::Task odom_task { [&] {
                              while (true) {
                                  uint32_t current_time = pros::millis();
                                  pf_motion_model.update();
                                  // pf_model.changeCustomParticle({
                                  // pf_motion_model.getPose().x,
                                  //                                 pf_motion_model.getPose().y,
                                  //                                 5_stDeg },
                                  //                               0);
                                  pros::c::task_delay_until(
                                    &current_time,
                                    to_msec(
                                      pf_motion_model.getTaskDeltaTime()));
                              }
                          },
                           "odom task" };
    pros::delay(100);

    pros::Task pf_task { [&] {
                            while (true) {
                                uint32_t current_time = pros::millis();
                                pf_model.update();
                                pros::c::task_delay_until(
                                  &current_time,
                                  to_msec(pf_model.getTaskDeltaTime()));
                            }
                        },
                         "pf task" };
    pros::delay(100);

    pros::Task smoother_task { [&] {
                                  while (true) {
                                      uint32_t current_time = pros::millis();
                                      smoother_model.update();

                                      // pf_model.changeCustomParticle({
                                      // smoother_model.getPose().x,
                                      //                                 smoother_model.getPose().y,
                                      //                                 10_stDeg
                                      //                                 },
                                      //                               0);
                                      pros::c::task_delay_until(
                                        &current_time,
                                        to_msec(
                                          smoother_model.getTaskDeltaTime()));
                                  }
                              },
                               "smoother task" };
    pros::delay(100);

    // initialize was performed
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

    // initialize screens
    screen::init();
}
