#include "apis.h"
//

#include "autos.h"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
#include "main.h"
#include "pros/imu.h"
#include "screen/screen.h"
#include "systems/drivetrain.h"
#include "systems/intake.h"
#include "systems/matchloader.h"
#include "systems/odom_retract.h"
#include "systems/piston.h"
#include "systems/sysid.h"
#include "systems/wings.h"
#include "tuning.h"

void findImuOrientation() {
    pros::imu_orientation_e_t imu_orientation = imu.get_physical_orientation();

    if (imu_orientation == pros::E_IMU_X_DOWN)
        std::cout << "E_IMU_X_DOWN" << std::endl;
    if (imu_orientation == pros::E_IMU_Y_DOWN)
        std::cout << "E_IMU_Y_DOWN" << std::endl;
    if (imu_orientation == pros::E_IMU_Z_DOWN)
        std::cout << "E_IMU_Z_DOWN" << std::endl;
    if (imu_orientation == pros::E_IMU_X_UP)
        std::cout << "E_IMU_X_UP" << std::endl;
    if (imu_orientation == pros::E_IMU_Y_UP)
        std::cout << "E_IMU_Y_UP" << std::endl;
    if (imu_orientation == pros::E_IMU_Z_UP)
        std::cout << "E_IMU_Z_UP" << std::endl;
}

void opcontrol() {
    // angular_kv_ks_tuner();
    // angular_ka_kp_ki_tuner();
    // return;

    // 24 - 0.88
    // 36 - 0.9
    // 48 - 1.02
    // 72 - 1.4

    turn_vel_pid_tuning();
	return;

    // drive_vel_pid_tuning();
    // drive_pid_tuning();

    // findImuOrientation();

    // return;
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
