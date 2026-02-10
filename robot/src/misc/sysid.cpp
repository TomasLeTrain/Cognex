#include "apis.h"
//
#include "blazing/utils.hpp"
#include "globals/blazing_globals.h"
#include "globals/device_globals.h"
#include "lyfast/system_identification.hpp"
#include "systems/sysid.h"

using namespace blazing;
using namespace blazing::lyfast;

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void kv_ks_tuner(
  std::string type,
  std::vector<lyfast::DifferentialSysIdVoltageCommands> voltage_commands,
  Time delta_time) {
    lyfast::DifferentialSysidData data;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {

            std::cout << "type: " << type << std::endl;

            data = lyfast::DifferentialSysid::calculate_kv_ks(voltage_commands,
                                                              drivetrain);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "kv/ks type: " << type << std::endl;
            lyfast::DifferentialSysid::printData(data, delta_time);
        }
        pros::delay(10);
    }
}

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void raw_ka_tuner(
  std::string type,
  std::vector<lyfast::DifferentialSysIdVoltageCommands> voltage_commands,
  lyfast::KvUnits left_Kv,
  lyfast::KsUnits left_Ks,
  lyfast::KvUnits right_Kv,
  lyfast::KsUnits right_Ks) {
    lyfast::DifferentialSysidData data;

    Time delta_time = 10_msec;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "type: " << type << std::endl;

            data = lyfast::DifferentialSysid::calculate_ka(voltage_commands,
                                                           drivetrain,
                                                           left_Kv,
                                                           left_Ks,
                                                           right_Kv,
                                                           right_Ks,
                                                           delta_time);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "type: " << type << std::endl;
            std::cout << "data" << std::endl;
            lyfast::DifferentialSysid::printData(data, delta_time);
        }
        pros::delay(10);
    }
}

void create_accel_data(lyfast::DifferentialSysIdVoltageCommands voltage_command,
                       std::string type) {
    Time delta_time = 10_msec;

    std::vector<lyfast::DifferentialSysIdVoltageCommands>
      accel_voltage_commands = { // linear movements
                                 // { u_step, u_step, 2_sec },
                                 voltage_command
      };

    lyfast::DifferentialSysidData accel_data;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "type: " << type << std::endl;

            accel_data =
              lyfast::DifferentialSysid::createData(accel_voltage_commands,
                                                    drivetrain,
                                                    delta_time);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "type: " << type << std::endl;
            std::cout << "accel_data: " << std::endl;
            std::cout << "u_step (l,r): " << voltage_command.left_voltage
                      << ", " << voltage_command.right_voltage << std::endl;
            lyfast::DifferentialSysid::printData(accel_data, delta_time);
        }
        pros::delay(10);
    }
}

void ka_kp_ki_tuner(std::string type,
                    lyfast::DifferentialSysIdVoltageCommands voltage_command,
                    double lambda_factor,
                    Time delta_time) {
    lyfast::DifferentialSysidData data;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "type: " << type << std::endl;

            data = lyfast::DifferentialSysid::calculate_ka_kp_ki_fopdt(
              voltage_command,
              drivetrain,
              delta_time,
              lambda_factor);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "type: " << type << std::endl;
            std::cout << "data: " << std::endl;
            lyfast::DifferentialSysid::printData(data, delta_time);
        }
        pros::delay(10);
    }
}

void linear_ka_kp_ki_tuner(Voltage u_step,
                           double lambda_factor,
                           Time accel_time,
                           Time delta_time) {

    ka_kp_ki_tuner("LINEAR",
                   { u_step, u_step, accel_time },
                   lambda_factor,
                   delta_time);
}

void angular_ka_kp_ki_tuner(Voltage u_step,
                            double lambda_factor,
                            Time accel_time,
                            Time delta_time) {
    ka_kp_ki_tuner("ANGULAR",
                   { u_step, -u_step, accel_time },
                   lambda_factor,
                   delta_time);
}

void linear_kv_ks_tuner(Time delta_time) {
    kv_ks_tuner("LINEAR",
                std::vector<lyfast::DifferentialSysIdVoltageCommands> {
                  // linear movements
                  { -0.1_volt, -0.1_volt, 600_msec  },
                  { 0.2_volt,  0.2_volt,  1000_msec },
                  { -0.3_volt, -0.3_volt, 1000_msec },
                  { 0.4_volt,  0.4_volt,  1000_msec },
                  { -0.5_volt, -0.5_volt, 1000_msec },
                  { 0.6_volt,  0.6_volt,  1000_msec },
                  { -0.7_volt, -0.7_volt, 1000_msec },
    },
                delta_time);
}

void angular_kv_ks_tuner(Time delta_time) {
    kv_ks_tuner("ANGULAR",
                std::vector<lyfast::DifferentialSysIdVoltageCommands> {
                  // linear movements
                  { 0.1_volt,  -0.1_volt, 600_msec  },
                  { -0.2_volt, 0.2_volt,  1000_msec },
                  { 0.3_volt,  -0.3_volt, 1000_msec },
                  { -0.4_volt, 0.4_volt,  1000_msec },
                  { 0.5_volt,  -0.5_volt, 1000_msec },
                  { -0.6_volt, 0.6_volt,  1000_msec },
                  { 0.7_volt,  -0.7_volt, 1000_msec },
    },
                delta_time);
}

void linear_raw_ka_tuner(lyfast::KvUnits left_Kv,
                         lyfast::KsUnits left_Ks,
                         lyfast::KvUnits right_Kv,
                         lyfast::KsUnits right_Ks) {
    std::vector<lyfast::DifferentialSysIdVoltageCommands>
      mixed_voltage_commands = {
          // linear movements
          { 0.5_volt,  0.5_volt,  500_msec },
          { 0.7_volt,  0.7_volt,  600_msec },
          { 0.2_volt,  0.2_volt,  600_msec },
          { 0.0_volt,  0.0_volt,  300_msec },
          { -0.7_volt, -0.7_volt, 800_msec },
          { -0.2_volt, -0.2_volt, 800_msec },
          { 0.0_volt,  0.0_volt,  300_msec },

          { 0.5_volt,  0.5_volt,  500_msec },
          { 0.7_volt,  0.7_volt,  600_msec },
          { 0.2_volt,  0.2_volt,  600_msec },
          { 0.0_volt,  0.0_volt,  300_msec },
          { -0.7_volt, -0.7_volt, 800_msec },
          { -0.2_volt, -0.2_volt, 800_msec },
          { 0.0_volt,  0.0_volt,  300_msec },
    };
    raw_ka_tuner("LINEAR",
                 mixed_voltage_commands,
                 left_Kv,
                 left_Ks,
                 right_Kv,
                 right_Ks);
}

void angular_raw_ka_tuner(lyfast::KvUnits left_Kv,
                          lyfast::KsUnits left_Ks,
                          lyfast::KvUnits right_Kv,
                          lyfast::KsUnits right_Ks) {
    std::vector<lyfast::DifferentialSysIdVoltageCommands>
      mixed_voltage_commands = {
          { 0.5_volt,  -0.5_volt, 500_msec },
          { 1.0_volt,  -1.0_volt, 400_msec },
          { -0.5_volt, 0.5_volt,  800_msec },
          { -0.2_volt, 0.2_volt,  800_msec },
          { 1.0_volt,  -1.0_volt, 400_msec },
          { -0.2_volt, 0.2_volt,  300_msec },

          { -0.5_volt, 0.5_volt,  500_msec },
          { -1.0_volt, 1.0_volt,  400_msec },
          { 0.5_volt,  -0.5_volt, 800_msec },
          { 0.2_volt,  -0.2_volt, 800_msec },
          { -1.0_volt, 1.0_volt,  400_msec },
          { 0.2_volt,  -0.2_volt, 300_msec },
    };
    raw_ka_tuner("LINEAR",
                 mixed_voltage_commands,
                 left_Kv,
                 left_Ks,
                 right_Kv,
                 right_Ks);
}

std::vector<MotorSysidData> calculate_intake_kv_ks(
  std::vector<lyfast::MotorSysidVoltageCommands> voltage_commands,
  pros::MotorGroup* motors,
  Time delta_time,
  Time steady_state_time) {
    std::vector<MotorSysidData> data;

    uint32_t int_delta_time = std::lround(to_msec(delta_time));

    for (auto [voltage, target_time, record] : voltage_commands) {
        motors->move_voltage(to_mvolt(voltage) * 12);

        auto start_time = blazing::now();
        uint32_t prev_time = pros::millis();

        LinearVelocity averageVelocities { 0 };
        Voltage averageVoltages { 0 };
        int samples = 0;

        while (!timeoutDone(target_time, start_time)) {
            // time at which we start to record data
            Time threshold_time =
              units::max(0_Fsec, target_time - steady_state_time);

            if (timeoutDone(threshold_time, start_time)) {

                //
                auto curr_vel =
                  blazing::get_group_velocity(motors, (1 / M_PI) * m, 600_rpm);

                averageVelocities += curr_vel;
                averageVoltages += voltage;

                samples++;
            }

            pros::c::task_delay_until(&prev_time, int_delta_time);
        }

        averageVelocities /= samples;
        averageVoltages /= samples;

        if (record) {
            data.emplace_back(averageVelocities, averageVoltages);
        }
    }

    auto [kv, ks] = lyfast::MotorGroupSysid::fit_kv_ks_data(data);

    auto new_kv = kv * rad / m;

    std::cout << "kv/ks: " << new_kv << " " << ks << std::endl;

    return data;
}
