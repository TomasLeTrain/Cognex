#include "apis.h"
//

#include "globals.h"
#include "globals/vexmaps_globals.h"
#include "health_daemon.h"
#include "main.h"
#include "screen/screen.h"
#include <mutex>

void initialize() {
    // initialize screens
    screen::init();

    // dont wait for task to finish?
    bool finished_reading_task = true;

    // start reading map same time as calibrating imu
    pros::Task([&finished_reading_task] {
        // give time for imu to start calibrating
        pros::delay(100);
        int map_reader_notif = screen::health::add_init_notif("Reading map");

        // give time for notifications to propagage
        pros::delay(50);

        // doesn't work???
        // map_reader.read_compressed("/usd/field_720_100_100.map.compressed");
        //
        // if (!map_reader.mapAvailable()) {
        //     // try to read map
        //     std::cout << "compressed map failed, try reading full map"
        //               << std::endl;
        //     map_reader.read("/usd/field_720_100_100.map");
        //     std::cout << "finished reading at " << pros::millis() <<
        //     std::endl;
        // }

        if (map_reader.mapAvailable()) {
            screen::health::update_init_notif_severity(map_reader_notif,
                                                       screen::health::succeed);
        } else {
            screen::health::update_init_notif_severity(
              map_reader_notif,
              screen::health::critical);
        }

        finished_reading_task = true;
    },"map reading");

    int imu_notif = screen::health::add_init_notif("calibrating imu");

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
        screen::health::update_init_notif_severity(imu_notif,
                                                   screen::health::critical);
    } else {
        screen::health::update_init_notif_severity(imu_notif,
                                                   screen::health::succeed);
    }

    // pros::delay(100);

    // wait for reading task to finish
    while (!finished_reading_task) {
        pros::delay(20);
    }

    int init_models_notif =
      screen::health::add_init_notif("initializing models");
    // initialize all models
    model_manager.init();
    screen::health::update_init_notif_severity(init_models_notif,
                                               screen::health::succeed);

    int init_executors_notif =
      screen::health::add_init_notif("initializing executors");
    // needed for async/chain motions to run
    async.init();
    chain.init();
    screen::health::update_init_notif_severity(init_executors_notif,
                                               screen::health::succeed);

    int init_tracker_notif =
      screen::health::add_init_notif("initializing tracker");
    // blazing tracker task
    pros::Task([&]() {
        while (true) {
            tracker.update();
            pros::delay(10);
        }
    },"blazing tracker");
    screen::health::update_init_notif_severity(init_tracker_notif,
                                               screen::health::succeed);

    // motion defaults

    int init_motion_defaults_notif =
      screen::health::add_init_notif("initializing motion defaults");

    // default a timeout
    mb.setTurnToModifier([](auto turnTo) {
        return turnTo.timeout(5_sec);
    });

    mb.setDistanceAtHeadingModifier([](auto distanceAtHeading) {
        return distanceAtHeading.timeout(5_sec);
    });

    mb.setMoveToModifier([](auto moveTo) {
        // return moveTo.customAngularLinearFunc(angular_linear_func);
        return moveTo.k_lat(0.15 * rad / m).timeout(3_sec);
    });

    mb.setBoomerangModifier([](auto boomerang) {
        // return boomerang.customAngularLinearFunc(angular_linear_func);
        // return boomerang.k_lat();
        return boomerang.k_lat(0.15 * rad / m, true).timeout(5_sec);
    });
    screen::health::update_init_notif_severity(init_motion_defaults_notif,
                                               screen::health::succeed);

    int init_health_daemon_notif =
      screen::health::add_init_notif("intializing health daemon",
                                     screen::health::warn);

    health_daemon::init_health_daemon();

    screen::health::update_init_notif_severity(init_health_daemon_notif,
                                               screen::health::succeed);

    screen::health::add_init_notif("finished initialize!",
                                   screen::health::succeed);

    pf_model.setReferenceModel(&smoother_model);

    // initialize was performed
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

    pros::Task([&] {
        while (true) {
            std::vector<std::pair<units::V2FPosition, float>> particles = {
                { pf_motion_model.getPose(), 0.01 },
                { pf_model.getPose(),        10   },
                { tracker.getPosition(),     30   },
                { smoother_model.getPose(),  100  }
            };

            pf_model.setCustomParticles(particles);
            pf_model.setCustomPrediction(smoother_model.getPose());

            pros::delay(10);
        }
    },"particle task");
}
