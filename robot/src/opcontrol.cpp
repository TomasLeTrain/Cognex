#include "apis.h"
//

#include "autos.h"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
#include "main.h"
#include "screen/screen.h"
#include "systems/drivetrain.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/piston.h"
#include "systems/sysid.h"
#include "systems/wings.h"
#include "tuning.h"

void opcontrol() {

    // goated linear pid:
    // kp: 6.9
    // ki: 0.3
    // kd: 9.9

    //    RobotSetPose(-48, -48, 180);
    //
    // intake::init(false);
    // long_goal_tuning();
    // return;

    // turn_pid_tuning();
    // drive_pid_tuning();
    // return;
    // autonomous();
    // return;

    // odom_offset_tuning();

    // matchloader::init(false);
    // turn_pid_tuning();
    // return;

    // RobotSetPose(10, 10, 0);

    // cancel any auton motions that could be currently running

    // linear_ka_kp_ki_tuner();
    // create_accel_data({ 0.5_volt, 0.5_volt, 2_sec }, "Linear");

    intake::init(true);
    matchloader::init(true);
    wings::init(true);
    odom_retract::init(true);

    intake::setDriverColorSortEnabled(false);

    // no need to initialize in auto
    drivetrain.setBrakeMode(pros::MotorBrake::coast);

    odom_retract::retractOdom();

    // RobotSetPose(-48, -48, 180);

    // matchloadTuning();
    // long_goal_tuning();
    //

    // runs exclusively inside opcontrol to guarantee it does not interfer with
    // autos (stopped automatically when not in driver mode)
    while (true) {
        base::driveUpdate();
        pros::delay(10);
    }
}
