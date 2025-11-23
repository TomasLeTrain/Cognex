#include "apis.h"
//

#include "autos.h"
#include "globals.h"
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
    // initialize tasks for each subsystem

    // autonomous();
    // return;

    // cancel any auton motions that could be currently running

    intake::init(true);
    matchloader::init(true);
    wings::init(true);
    odom_retract::init(true);

    // no need to initialize in auto
    base::init();

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
    odom_retract::set(piston_state_t::active);

    RobotSetPose(36, -36, 0);

    // while (true) {
    //     auto blazing_position = tracker.getPosition();
    //     auto blazing_theta = tracker.getAngle();
    //
    //     // auto vexmaps_pose = model_manager.getPose();
    //
    //     // printf("tf\n");
    //     // screen::health::set_console_text(std::format(
    //     //   "blazing: {:.2f} {:.2f} {:.2f}\nvexmaps: {:.2f} {:.2f} {:.2f}",
    //     //   blazing_position.x.convert(in),
    //     //   blazing_position.y.convert(in),
    //     //   blazing_theta.convert(deg),
    //     //   vexmaps_pose.x.convert(in),
    //     //   vexmaps_pose.y.convert(in),
    //     //   vexmaps_pose.orientation.convert(deg)));
    //
    //     // maybe unneeded?
    //     pros::delay(50);
    // }
}
