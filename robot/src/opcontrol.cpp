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
#include "systems/wings.h"
#include "tuning.h"

void opcontrol() {
    // autonomous();
    // return;

    // odom_offset_tuning();

    // matchloader::init(false);
    // turn_pid_tuning();
    // return;

    // RobotSetPose(10, 10, 0);

    // cancel any auton motions that could be currently running

    RobotSetPose(-48, 0, 0);

    intake::init(true);
    matchloader::init(true);
    wings::init(true);
    odom_retract::init(true);

    intake::setDriverColorSortEnabled(false);

    // no need to initialize in auto
    drivetrain.setBrakeMode(pros::MotorBrake::coast);
    base::init();
    odom_retract::retractOdom();
}
