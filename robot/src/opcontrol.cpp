#include "apis.h"
//

#include "autos.h"
#include "globals.h"
#include "globals/blazing_globals.h"
#include "globals/device_globals.h"
#include "globals/vexmaps_globals.h"
#include "lyfast/system_identification.hpp"
#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/imu.h"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
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

    // matchloader::init(false);
    // turn_vel_pid_tuning();
    // drive_vel_pid_tuning();
    // return;
    // turn_vel_pid_tuning();
    // drive_vel_pid_tuning();

    // intake::init(false);
    // matchloader::init(false);
    // wings::init(false);
    // odom_retract::init(false);
    // long_goal_tuning();
    // return;
    //

    // drive_pid_tuning();

    // findImuOrientation();

    // pros::delay(2000);
    // pros::MotorGroup bottom_intake_group({ bottom_motor.get_port() },
    //                                      pros::MotorGears::blue,
    //                                      pros::v5::MotorUnits::rotations);
    //
    // std::vector<blazing::lyfast::MotorSysidData> data =
    // calculate_intake_kv_ks(
    //   std::vector<blazing::lyfast::MotorSysidVoltageCommands> {
    //     { .voltage = 0.05_volt,  .time = 800_msec },
    //     { .voltage = -0.05_volt, .time = 800_msec },
    //     { .voltage = 0.1_volt,   .time = 800_msec },
    //     { .voltage = -0.1_volt,  .time = 800_msec },
    //     { .voltage = 0.2_volt,   .time = 800_msec },
    //     { .voltage = -0.2_volt,  .time = 800_msec },
    //     { .voltage = 0.3_volt,   .time = 800_msec },
    //     { .voltage = -0.3_volt,  .time = 800_msec },
    //     { .voltage = 0.4_volt,   .time = 800_msec },
    //     { .voltage = -0.4_volt,  .time = 800_msec },
    //     { .voltage = 0.5_volt,   .time = 800_msec },
    //     { .voltage = -0.5_volt,  .time = 800_msec },
    //     { .voltage = 0.6_volt,   .time = 800_msec },
    //     { .voltage = -0.6_volt,  .time = 800_msec },
    //     { .voltage = 0.7_volt,   .time = 800_msec },
    //     { .voltage = -0.7_volt,  .time = 800_msec },
    //     { .voltage = 0.8_volt,   .time = 800_msec },
    //     { .voltage = -0.8_volt,  .time = 800_msec },
    // },
    //   &bottom_intake_group,
    //   10_msec,
    //   50_msec);
    //
    // std::cout << "\\left[";
    // for (int i = 0; i < data.size(); i++) {
    //     std::cout << "\\left(" << data[i].voltage.internal() << ","
    //               << data[i].velocity.convert(mps) << "\\right)";
    //     // doesn't print comma for last point
    //     if (i < data.size() - 1) std::cout << ",";
    // }
    // std::cout << "\\right]" << std::endl;

    // return;

    // return;

    // RobotSetPose(-48, -24, 0);

    // todo list:
    // 4. visualizer test?
    // 5. turn pid tuning
    // 6. linear pid tuning
    // 7. autos

    // drive_vel_pid_tuning();
    // return;

    // turn_vel_pid_tuning();
    // return;

	// odom_offset_tuning();
	// return;

    // linear_kv_ks_tuner();
    // angular_ka_kp_ki_tuner();
    // angular_kv_ks_tuner();
    // return;

    //
    // turn_pid_tuning();
    // odom_offset_tuning();
    // odom_diameter_tuning();
    // return;

    // turn_vel_pid_tuning();
    // drive_vel_pid_tuning();
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

    // RobotSetPose(48, 48, 0);
    odom_retract::retractOdom();

   // RobotSetPose(-48, 48, 0);
    //
    //
    //
    //

    //
    //    pros::delay(3000);
    // pf_model.setDisabled(true);

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
