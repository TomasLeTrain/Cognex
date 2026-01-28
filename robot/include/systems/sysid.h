#pragma once

#include "apis.h"
//

void kv_ks_tuner(std::string type,
                 std::vector<blazing::lyfast::DifferentialSysIdVoltageCommands>
                   voltage_commands,
                 Time delta_time = 10_msec);

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void raw_ka_tuner(std::string type,
                  std::vector<blazing::lyfast::DifferentialSysIdVoltageCommands>
                    voltage_commands,
                  blazing::lyfast::KvUnits left_Kv,
                  blazing::lyfast::KsUnits left_Ks,
                  blazing::lyfast::KvUnits right_Kv,
                  blazing::lyfast::KsUnits right_Ks);
void create_accel_data(
  blazing::lyfast::DifferentialSysIdVoltageCommands voltage_command,
  std::string type);

void ka_kp_ki_tuner(
  std::string type,
  blazing::lyfast::DifferentialSysIdVoltageCommands voltage_command,
  double lambda_factor,
  Time delta_time = 10_msec);

void linear_ka_kp_ki_tuner(Voltage u_step = 0.5_volt,
                           double lambda_factor = 0.6,
                           Time accel_time = 2_sec,
                           Time delta_time = 10_msec);
void angular_ka_kp_ki_tuner(Voltage u_step = 0.5_volt,
                            double lambda_factor = 0.6,
                            Time accel_time = 2_sec,
                            Time delta_time = 10_msec);
void linear_kv_ks_tuner(Time delta_time = 10_msec);
void angular_kv_ks_tuner(Time delta_time = 10_msec);
void linear_raw_ka_tuner(blazing::lyfast::KvUnits left_Kv,
                         blazing::lyfast::KsUnits left_Ks,
                         blazing::lyfast::KvUnits right_Kv,
                         blazing::lyfast::KsUnits right_Ks);
void angular_raw_ka_tuner(blazing::lyfast::KvUnits left_Kv,
                          blazing::lyfast::KsUnits left_Ks,
                          blazing::lyfast::KvUnits right_Kv,
                          blazing::lyfast::KsUnits right_Ks);
