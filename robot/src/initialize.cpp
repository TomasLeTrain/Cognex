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
    model_manager.init();

    // needed for async/chain motions to run
    async.init();
    chain.init();

	// blazing tracker task
    pros::Task([&]() {
        while (true) {
            tracker.update();
            pros::delay(10);
        }
    });

    // motion defaults

    // default a timeout
    mb.setTurnToModifier([](auto turnTo) {
        return turnTo.timeout(5_sec);
    });

    mb.setDistanceAtHeadingModifier([](auto distanceAtHeading) {
        return distanceAtHeading.timeout(5_sec);
    });

    mb.setMoveToModifier([](auto moveTo) {
        // return moveTo.customAngularLinearFunc(angular_linear_func);
        return moveTo.k_lat(0.3 * rad / m).timeout(5_sec);
    });

    mb.setBoomerangModifier([](auto boomerang) {
        // return boomerang.customAngularLinearFunc(angular_linear_func);
        // return boomerang.k_lat();
        return boomerang.k_lat(0.2 * rad / m, true).timeout(7_sec);
    });

    // initialize was performed
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

    // initialize screens
    screen::init();
}
