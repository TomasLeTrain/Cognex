#include "apis.h"
//

#include "autos.h"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/vexmaps_globals.h"
#include "main.h"
#include "screen/screen.h"
#include "systems/drivetrain.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/piston.h"
#include "systems/wings.h"
#include "tuning.h"

void opcontrol() {
    // autonomous();
    // return;

    // initialize tasks for each subsystem
    // odom_diameter_tuning();
    // odom_offset_tuning();
    // return;

    // autonomous();
    // return;
    // turn_pid_tuning();
    // drive_pid_tuning();
    // return;

    // 90 turn:  2.6 3.35
    // 135 turn: 2.6 3.95
    // 180 turn: 2.6 4.45

    // cancel any auton motions that could be currently running

    intake::init(true);
    matchloader::init(true);
    wings::init(true);
    odom_retract::init(true);

    intake::setDriverColorSortEnabled(false);

    // no need to initialize in auto
    drivetrain.setBrakeMode(pros::MotorBrake::coast);
    base::init();
    odom_retract::retractOdom();

    // set screen in case its different
    // screen::setScreen(&screen::bouncing_dvd_screen::screen);

    // bool print_info = true;

    // pros::Task smoother_task { [&] {
    //     while (print_info) {
    //         int start_time = pros::millis();
    //         printf("start generation\nstart distances\nend distances\nstart "
    //                "parti"
    //                "cles"
    //                "\n");
    //
    //         printf("%.1f %.1f %.1f\n",
    //                pf_motion_model.getPose().x.convert(in),
    //                pf_motion_model.getPose().y.convert(in),
    //                0.0);
    //         if (pf_model.getConfidence() != std::nullopt) {
    //             printf("%.1f %.1f %.1f\n",
    //                    pf_model.getPose().x.convert(in),
    //                    pf_model.getPose().y.convert(in),
    //                    5.0);
    //         }
    //         printf("%.1f %.1f %.1f\n",
    //                smoother_model.getPose().x.convert(in),
    //                smoother_model.getPose().y.convert(in),
    //                10.0);
    //
    //         printf("end particles\ntotal weight: 0, time taken: 30000, "
    //                "timestamp:"
    //                " %d\n",
    //                start_time);
    //         printf("things done:1,1,0,%d\n", 16384);
    //         printf("prediction:%.1f,%.1f,%.1f\n",
    //                smoother_model.getPose().x.convert(in),
    //                smoother_model.getPose().y.convert(in),
    //                smoother_model.getPose().orientation.convert(deg));
    //         printf("end generation\n");
    //
    //         pros::delay(10);
    //     }
    // } };

    // RobotSetPose({ 2_tile, 2_tile, 90_stDeg });
    // RobotSetPose(0, 0, 0);
    // pros::delay(100);
    // LaserResets({ &back_laser_model, &right_laser_model });

    // odom_tuning();

    // move odom up automatically

    // intake::setDriverColorSortEnabled(false);

    // RobotSetPose(-46.471, 18.535, 0);
    //   while (true) {
    //       std::cout << std::format(
    //         "{:.2f},{:.2f},{:.2f}",
    //         model_manager.getPose().x.convert(in),
    //         model_manager.getPose().y.convert(in),
    //         model_manager.getPose().orientation.convert(deg)) << std::endl;
    // pros::delay(1000);
    //   }

    // RobotSetPose({ 49_in, -16.8_in, 200 * deg });

    // vexmaps::DistanceSensorConfig new_config = distance_sensor_config;
    //
    // new_config.maxDistanceDifference = 10_in;

    // change front model to accept larger changes due to drift in wheels
    // front_laser_model.setConfig(new_config);
    // left_laser_model.setConfig(new_config);
    // right_laser_model.setConfig(new_config);
    // back_laser_model.setConfig(new_config);

    // while (true) {
    //     screen::health::set_console_text(
    //       std::format("vexmaps forwards travel: {:.4f}_in\n"
    //                   "pf model forwards travel: {:.4f}_in\n"
    //                   "blazing forwards travel: {:.4f}_in\n",
    //                   vexmaps_tracker.getForwardTravel().convert(in),
    //                   pf_motion_model.getForwardTravel().convert(in),
    //                   tracker.getForwardTravel().convert(in)));
    //
    //     pros::delay(50);
    // }
}
