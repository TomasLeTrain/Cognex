#include "apis.h"
//

#include "blazing/utils.hpp"
#include "controller_ui/controller_auton_selector.h"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/config.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
#include "health_daemon.h"
#include "main.h"
#include "screen/screen.h"
#include "systems/intake.h"
#include <mutex>
#include <string>

// task with critical timing that allows gathering consistent data
void timeCriticalTask() {
    // code section taken from sylib:
    // https://github.com/sy1vi3/sylib/blob/5b2eef6812f65b4305b74e415d3a570c10213fff/src/sylib/system.cpp

    // A 1ms loop will actually take around 1040 or 960 microseconds, always
    // alternating. Over 3ms, the total length of time in micros should be
    // either around 3040 or 960 Daemon needs to start on a cycle to be
    // directly opposite of vexBackgroundProcessing()
    // vexBackgroundProcessing always runs after a short cycle, meaning the
    // sylib daemon needs to start after a long cycle Values offset by 20 to
    // give room for error, the groupings are very tight so it shouldnt
    // matter

    constexpr std::uint64_t LONG_MICROS_CYCLE_LENGTH = 1040 - 20;
    constexpr std::uint64_t AVERAGE_MICROS_CYCLE_LENGTH = 1000;
    constexpr std::uint64_t DIFFERENCE_BETWEEN_AVERAGE_AND_LONG =
      LONG_MICROS_CYCLE_LENGTH - AVERAGE_MICROS_CYCLE_LENGTH;

    uint32_t systemTime = pros::millis();
    uint32_t detectorPreviousTime = pros::millis();
    uint64_t systemTimeMicros = pros::micros();
    uint64_t prevMicros = systemTimeMicros;

    int frameCount = 0;

    std::cout << "starting task: " << pros::micros() << std::endl;

    do {
        systemTimeMicros = pros::micros();
        detectorPreviousTime = systemTime;
        prevMicros = systemTimeMicros;
        pros::Task::delay_until(&systemTime, 3);
    } while (
      (pros::micros() - prevMicros) >
      (((systemTime - detectorPreviousTime) * AVERAGE_MICROS_CYCLE_LENGTH) -
       DIFFERENCE_BETWEEN_AVERAGE_AND_LONG));
    /*
    NOW WE'RE TIMED CORRECTLY, STARTING DAEMON
    */

    std::cout << "timed correctly: " << pros::micros() << '\n';

    while (1) {
        {
            frameCount++;

            // do stuff here
            if (frameCount % 5 == 0) {
                uint32_t curr_time = pros::millis();
                tracker.update();

                // TODO: chopped since its not guarnteed to have updated ->
                // might introduce input delay
                LinearVelocity forwards_velocity =
                  model_manager.getLocalVelocityVector().x;

                // not using forwards velocity since its offset is not
                // guaranteed to be zero
                // LinearVelocity forwards_velocity =
                //   toLinear(from_degps(forwards_odom_rotation.get_velocity()),
                //            1.991_in);

                // imu up orientation
                AngularVelocity angular_velocity =
                  from_degps(imu.get_gyro_rate().z);

                // auto motor_left_vel =
                //   blazing::getGroupVelocity(&left_motors,
                //                             drivetrain_config.wheel_diameter,
                //                             drivetrain_config.rpm);
                //
                // auto motor_right_vel =
                //   blazing::getGroupVelocity(&right_motors,
                //                             drivetrain_config.wheel_diameter,
                //                             drivetrain_config.rpm);
                //
                // auto linear_vel = (motor_right_vel + motor_left_vel) / 2.0;
                //
                // TODO: temporary
                // LinearVelocity left_vel =
                //   linear_vel -
                //   angular_velocity * drivetrain_config.track_radius / rad;
                // LinearVelocity right_vel =
                //   linear_vel +
                //   angular_velocity * drivetrain_config.track_radius / rad;

                LinearVelocity left_vel =
                  forwards_velocity -
                  angular_velocity * drivetrain_config.track_radius / rad;
                LinearVelocity right_vel =
                  forwards_velocity +
                  angular_velocity * drivetrain_config.track_radius / rad;

                LeftRightSpeeds measurement { left_vel, right_vel };

                // update plant
                drivetrain_plant.setMeasurement(measurement);
                drivetrain_plant.updateToTimestamp(curr_time);
                auto curr_drivetrain_voltages =
                  drivetrain_plant.getCommandedVoltages();

                left_motors.move_voltage(
                  12 * to_mvolt(curr_drivetrain_voltages.left_voltage));
                right_motors.move_voltage(
                  12 * to_mvolt(curr_drivetrain_voltages.right_voltage));
                // std::cout << "moving: "
                //           << curr_drivetrain_voltages.left_voltage.internal()
                //           << " "
                //           <<
                //           curr_drivetrain_voltages.right_voltage.internal()
                //           << std::endl;
            }

            pros::Task::delay_until(&systemTime, 2);
        }
    }
}

void startTimeCriticalTask() {
    static bool daemonStarted = false;
    if (!daemonStarted) {
        pros::Task managerTask(timeCriticalTask,
                               15, // very high priority
                               TASK_STACK_DEPTH_DEFAULT,
                               "time critical task");
        daemonStarted = true;
    }
}

void initialize() {
    // initialize screens
    screen::init();

    // initialize controller screen control
    // controller_ui::init();

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

    // initialize task only after imu has been init since tracker requires it
    startTimeCriticalTask();

    // make the imu return data as fast as possible?
    // imu.set_data_rate(5);

    // give time for screen to update
    // pros::delay(50);

    int init_models_notif =
      screen::health::add_init_notif("initializing models");

    // initialize all models
    model_manager.init();

    screen::health::update_init_notif_severity(init_models_notif,
                                               screen::health::succeed);

    // give time for screen to update
    // pros::delay(50);

    int init_executors_notif =
      screen::health::add_init_notif("initializing executors");
    // pros::delay(50);

    // needed for async/chain motions to run
    async.init();
    chain.init();

    // Each temperature level limits the motor current:
    // 1 = 50% current,
    // 2 = 25% current,
    // 3 = 12.5% current,
    // 4 = 0% current.

    // pros::delay(50);

    screen::health::update_init_notif_severity(init_executors_notif,
                                               screen::health::succeed);
    // pros::delay(50);

    int init_tracker_notif =
      screen::health::add_init_notif("initializing tracker");
    // pros::delay(50);

    screen::health::update_init_notif_severity(init_tracker_notif,
                                               screen::health::succeed);

    // motion defaults

    int init_motion_defaults_notif =
      screen::health::add_init_notif("initializing motion defaults");

    // actually vel but just set them to change all autos
    mb.setTurnToModifier([](auto turnTo) {
        std::ignore =
          turnTo
            ->velocity_based(true)
            // speecifically uses turn heading pid instead of drive pid
            // .withAngularVelocityFeedbackController(turn_heading_vel_pid)
            .timeout(3_sec);
    });

    mb.setArcModifier([](auto arc) -> auto {
        std::ignore =
          arc
            ->velocity_based(true)
            // speecifically uses turn heading pid instead of drive pid
            // .withAngularVelocityFeedbackController(turn_heading_vel_pid)
            .timeout(3_sec);
    });

    //
    mb.setDistanceAtHeadingModifier([](auto distanceAtHeading) {
        std::ignore = distanceAtHeading->velocity_based(true).timeout(3_sec);
    });

    mb.setMoveToModifier([](auto moveTo) {
        std::ignore = moveTo->velocity_based(true)
                        .customAngularLinearFunc(angular_linear_func)
                        .k_lat(std::nullopt)
                        .timeout(3_sec);
    });

    mb.setBoomerangModifier([](auto boomerang) {
        std::ignore = boomerang->velocity_based(true)
                        .customAngularLinearFunc(angular_linear_func)
                        .k_lat(std::nullopt, true)
                        .timeout(5_sec);
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

    // std::cout << "set pf reference :" << &smoother_model << std::endl;
    // pros::delay(50);

    // sets reference for mcl
    pf_model.setReferenceModel(&smoother_model);

    // std::cout << "rumble" << std::endl;
    // pros::delay(50);

    // initialize was performed
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");

    // std::cout << "make pf thingy" << std::endl;
    // pros::delay(50);

    // taks to update custom particles in mcl logging
    pros::Task(
      [&] {
          while (true) {
              // std::cout << "pf task" << std::endl;
              std::vector<std::pair<units::V2FPosition, float>> particles = {
                  { pf_motion_model.getPose(), 0.01 },
                  { pf_model.getPose(),        10   },
                  { tracker.getPosition(),     30   },
                  // { smoother_model.getPose(),  80   },
                  { model_manager.getPose(),   80   }
              };

              pf_model.setCustomParticles(particles);
              pf_model.setCustomPrediction(smoother_model.getPose());

              std::stringstream str;
              str << std::fixed << std::setprecision(5);
              str << "vel:";
              str << model_manager.getLocalVelocityVector().x.convert(inps);
              str << ",";
              str << model_manager.getLocalVelocityVector().y.convert(inps);
              str << "\nalphas:";
              str << smoother_model.getAlphas().x.internal();
              str << ",";
              str << smoother_model.getAlphas().y.internal();

              pf_model.setCustomData(str.str());

              pros::delay(10);
          }
      },
      "particle task");

    // pros::Task(
    //   [&] {
    //       while (true) {
    //           screen::health::set_console_text(
    //             std::format("position: {:.4f}",
    //                         intake::lever::getLeverPosition()));
    //
    //           // 0.7411
    //
    //           // screen::health::set_console_text(
    //           //   std::format("vexmaps pose: {:.4f} {:.4f}\n"
    //           //               "motion model pose: {:.4f} {:.4f}\n"
    //           //               "blazing pose: {:.4f} {:.4f} {:.4f}\n",
    //           //               vexmaps_tracker.getPosition().x.convert(in),
    //           //               vexmaps_tracker.getPosition().y.convert(in),
    //           //               pf_motion_model.getPose().x.convert(in),
    //           //               pf_motion_model.getPose().y.convert(in),
    //           //               tracker.getPosition().x.convert(in),
    //           //               tracker.getPosition().y.convert(in),
    //           //               tracker.getAngle().convert(deg)));
    //
    //           pros::delay(50);
    //       }
    //   },
    //   "health screen particle task");
}
