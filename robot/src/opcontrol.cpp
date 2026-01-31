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
    // autonomous();
    // return;

    // RobotSetPose(-48, -24, 0);

    // todo list:
    // 4. visualizer test?
    // 5. turn pid tuning
    // 6. linear pid tuning
    // 7. autos

    // turn_pid_tuning();
    // drive_pid_tuning();
    // return;

    //
    // turn_pid_tuning();
    // odom_offset_tuning();
    // odom_diameter_tuning();
    // return;

    // matchloader::init(false);
    // return;

    // RobotSetPose(10, 10, 0);

    // cancel any auton motions that could be currently running

    intake::init(true);
    matchloader::init(true);
    wings::init(true);
    odom_retract::init(true);

    intake::setDriverColorSort(false);

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
