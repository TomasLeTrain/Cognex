#include "apis.h"
//
#include "blazing/utils.hpp"
#include "globals/blazing_globals.h"
#include "globals/device_globals.h"
#include "lyfast/sysid/system_identification.hpp"
#include "systems/sysid.h"

using namespace blazing;
using namespace blazing::lyfast;

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void genericTuner(
  const std::string& type,
  Time delta_time,
  std::function<lyfast::sysid::DifferentialData()> gatherData,
  std::function<void(const lyfast::sysid::DifferentialData&)> processData) {
    lyfast::sysid::DifferentialData data;

    while (true) {
        drivetrain.moveTank(0_volt, 0_volt);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            std::cout << "type: " << type << std::endl;

            data = gatherData();

            processData(data);
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            std::cout << "type: " << type << std::endl;
            std::cout << "data: " << std::endl;
            lyfast::sysid::DifferentialUtils::printData(data, delta_time);
            // printDrivetrainData();
        }
        pros::delay(10);
    }
}

void kv_ks_tuner(const std::string& type,
                 const std::vector<lyfast::sysid::DifferentialVoltageCommand>&
                   voltage_commands,
                 bool use_measured_voltage,
                 Time steady_state_time,
                 Time delta_time) {
    using namespace lyfast::sysid;
    // function params outlive the genericTuner function, so capturing
    // them by reference should fine

    genericTuner(
      type,
      delta_time,
      [&] -> DifferentialData {
          return DifferentialUtils::generate_kv_ks_data(voltage_commands,
                                                        drivetrain,
                                                        delta_time,
                                                        steady_state_time,
                                                        use_measured_voltage);
      },
      [](const DifferentialData& data) {
          DifferentialUtils::calculate_kv_ks(data);
      });
}

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void raw_ka_tuner(const std::string& type,
                  const std::vector<lyfast::sysid::DifferentialVoltageCommand>&
                    voltage_commands,
                  lyfast::KvUnits<LinearVelocity> left_Kv,
                  lyfast::KsUnits left_Ks,
                  lyfast::KvUnits<LinearVelocity> right_Kv,
                  lyfast::KsUnits right_Ks,
                  bool use_measured_voltage,
                  Time delta_time) {
    using namespace lyfast::sysid;
    // function params outlive the genericTuner function, so capturing
    // them by reference should fine
    genericTuner(
      type,
      delta_time,
      [&] {
          return DifferentialUtils::generateData(voltage_commands,
                                                 drivetrain,
                                                 delta_time,
                                                 use_measured_voltage);
      },
      [&](const DifferentialData& data) {
          DifferentialUtils::calculate_ka(data,
                                          left_Kv,
                                          left_Ks,
                                          right_Kv,
                                          right_Ks,
                                          delta_time);
      });
}

void create_accel_data(
  const lyfast::sysid::DifferentialVoltageCommand& voltage_command,
  const std::string& type,
  Time delta_time) {
    using namespace lyfast::sysid;
    // function params outlive the genericTuner function, so capturing
    // them by reference should fine
    genericTuner(
      type,
      delta_time,
      [&] {
          return DifferentialUtils::generateData({ voltage_command },
                                                 drivetrain,
                                                 delta_time);
      },
      [](const DifferentialData& data) {});
}

void ka_kp_ki_tuner(
  const std::string& type,
  const lyfast::sysid::DifferentialVoltageCommand& voltage_command,
  double lambda_factor,
  bool use_measured_voltage,
  Time delta_time) {
    using namespace lyfast::sysid;
    // function params outlive the genericTuner function, so capturing
    // them by reference should fine
    genericTuner(
      type,
      delta_time,
      [&] {
          return DifferentialUtils::generateData({ voltage_command },
                                                 drivetrain,
                                                 delta_time,
                                                 use_measured_voltage);
      },
      [&](const DifferentialData& data) {
          DifferentialUtils::calculate_ka_kp_ki_fopdt(data,
                                                      delta_time,
                                                      lambda_factor);
      });
}

void linear_ka_kp_ki_tuner(Voltage u_step,
                           double lambda_factor,
                           Time accel_time,
                           bool use_measured_voltage,
                           Time delta_time) {
    ka_kp_ki_tuner("LINEAR",
                   { u_step, u_step, accel_time },
                   lambda_factor,
                   use_measured_voltage,
                   delta_time);
}

void angular_ka_kp_ki_tuner(Voltage u_step,
                            double lambda_factor,
                            Time accel_time,
                            bool use_measured_voltage,
                            Time delta_time) {
    ka_kp_ki_tuner("ANGULAR",
                   { u_step, -u_step, accel_time },
                   lambda_factor,
                   use_measured_voltage,
                   delta_time);
}

void linear_kv_ks_tuner(bool use_measured_voltage,
                        Time steady_state_time,
                        Time delta_time) {

    std::vector<lyfast::sysid::DifferentialVoltageCommand> commands = {
        { -0.1_volt, -0.1_volt, 600_msec },
        { 0.0_volt, 0.0_volt, 500_msec, false },
    };

    // from 0.2 to 0.9, alternating sign
    for (int i = 2; i < 10; i++) {
        Voltage curr = Voltage(i / 10.0) * (i % 2 == 0 ? 1 : -1);
        commands.push_back({ curr, curr, 1000_msec });
        commands.push_back({ 0.0_volt, 0.0_volt, 500_msec, false });
    }

    kv_ks_tuner("LINEAR",
                // linear movements
                commands,
                use_measured_voltage,
                steady_state_time,
                delta_time);
}

void angular_kv_ks_tuner(bool use_measured_voltage,
                         Time steady_state_time,
                         Time delta_time) {
    kv_ks_tuner("ANGULAR",
                std::vector<lyfast::sysid::DifferentialVoltageCommand> {
                  // increasing intensity, changing sign
                  { 0.1_volt,  -0.1_volt, 600_msec  },
                  { -0.2_volt, 0.2_volt,  1000_msec },
                  { 0.3_volt,  -0.3_volt, 1000_msec },
                  { -0.4_volt, 0.4_volt,  1000_msec },
                  { 0.5_volt,  -0.5_volt, 1000_msec },
                  { -0.6_volt, 0.6_volt,  1000_msec },
                  { 0.7_volt,  -0.7_volt, 1000_msec },
                  { -0.8_volt, 0.8_volt,  1300_msec },
                  { 0.9_volt, -0.9_volt,  1300_msec },
    },
                use_measured_voltage,
                steady_state_time,
                delta_time);
}

void linear_raw_ka_tuner(lyfast::KvUnits<LinearVelocity> left_Kv,
                         lyfast::KsUnits left_Ks,
                         lyfast::KvUnits<LinearVelocity> right_Kv,
                         lyfast::KsUnits right_Ks,
                         bool use_measured_voltage,
                         Time delta_time) {
    std::vector<lyfast::sysid::DifferentialVoltageCommand>
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
                 right_Ks,
                 use_measured_voltage,
                 delta_time);
}

void angular_raw_ka_tuner(lyfast::KvUnits<LinearVelocity> left_Kv,
                          lyfast::KsUnits left_Ks,
                          lyfast::KvUnits<LinearVelocity> right_Kv,
                          lyfast::KsUnits right_Ks,
                          bool use_measured_voltage,
                          Time delta_time) {
    std::vector<lyfast::sysid::DifferentialVoltageCommand>
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
    raw_ka_tuner("ANGULAR",
                 mixed_voltage_commands,
                 left_Kv,
                 left_Ks,
                 right_Kv,
                 right_Ks,
                 use_measured_voltage,
                 delta_time);
}

// std::vector<MotorSysidData> calculate_intake_kv_ks(
//   std::vector<lyfast::MotorSysidVoltageCommands> voltage_commands,
//   pros::MotorGroup* motors,
//   Time delta_time,
//   Time steady_state_time) {
//     std::vector<MotorSysidData> data;
//
//     uint32_t int_delta_time = std::lround(to_msec(delta_time));
//
//     for (auto [voltage, target_time, record] : voltage_commands) {
//         motors->move_voltage(to_mvolt(voltage) * 12);
//
//         auto start_time = blazing::now();
//         uint32_t prev_time = pros::millis();
//
//         LinearVelocity averageVelocities { 0 };
//         Voltage averageVoltages { 0 };
//         int samples = 0;
//
//         while (!timeoutDone(target_time, start_time)) {
//             // time at which we start to record data
//             Time threshold_time =
//               units::max(0_Fsec, target_time - steady_state_time);
//
//             if (timeoutDone(threshold_time, start_time)) {
//
//                 //
//                 auto curr_vel =
//                   blazing::get_group_velocity(motors, (1 / M_PI) * m,
//                   600_rpm);
//
//                 averageVelocities += curr_vel;
//                 averageVoltages += voltage;
//
//                 samples++;
//             }
//
//             pros::c::task_delay_until(&prev_time, int_delta_time);
//         }
//
//         averageVelocities /= samples;
//         averageVoltages /= samples;
//
//         if (record) {
//             data.emplace_back(averageVelocities, averageVoltages);
//         }
//     }
//
//     auto [kv, ks] = lyfast::MotorGroupSysid::fit_kv_ks_data(data);
//
//     auto new_kv = kv * rad / m;
//
//     std::cout << "kv/ks: " << new_kv << " " << ks << std::endl;
//
//     return data;
// }
