#pragma once

#include "apis.h"
#include "lyfast/sysid/system_identification.hpp"
//

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void genericTuner(
  const std::string& type,
  Time delta_time,
  std::function<blazing::lyfast::sysid::DifferentialData()> gatherData,
  std::function<void(const blazing::lyfast::sysid::DifferentialData&)>
    processData);

void kv_ks_tuner(
  const std::string& type,
  const std::vector<blazing::lyfast::sysid::DifferentialVoltageCommand>&
    voltage_commands,
  bool use_measured_voltage = false,
  Time steady_state_time = 150_msec,
  Time delta_time = 10_msec);

// allows running tuning routine multiple times
// press A to run routine, X to get raw data
void raw_ka_tuner(
  const std::string& type,
  const std::vector<blazing::lyfast::sysid::DifferentialVoltageCommand>&
    voltage_commands,
  blazing::lyfast::KvUnits<LinearVelocity> left_Kv,
  blazing::lyfast::KsUnits left_Ks,
  blazing::lyfast::KvUnits<LinearVelocity> right_Kv,
  blazing::lyfast::KsUnits right_Ks,
  bool use_measured_voltage = true,
  Time delta_time = 10_msec);

void create_accel_data(
  const blazing::lyfast::sysid::DifferentialVoltageCommand& voltage_command,
  const std::string& type,
  Time delta_time = 10_msec);

void ka_kp_ki_tuner(
  const std::string& type,
  const blazing::lyfast::sysid::DifferentialVoltageCommand& voltage_command,
  double lambda_factor,
  bool use_measured_voltage = false,
  Time delta_time = 10_msec);

void linear_ka_kp_ki_tuner(Voltage u_step = 0.5_volt,
                           double lambda_factor = 0.6,
                           Time accel_time = 2_sec,
                           bool use_measured_voltage = false,
                           Time delta_time = 10_msec);

void angular_ka_kp_ki_tuner(Voltage u_step = 0.5_volt,
                            double lambda_factor = 0.6,
                            Time accel_time = 2_sec,
                            bool use_measured_voltage = false,
                            Time delta_time = 10_msec);

void linear_kv_ks_tuner(bool use_measured_voltage = false,
                        Time steady_state_time = 100_msec,
                        Time delta_time = 10_msec);

void angular_kv_ks_tuner(bool use_measured_voltage = false,
                         Time steady_state_time = 100_msec,
                         Time delta_time = 10_msec);

void linear_raw_ka_tuner(blazing::lyfast::KvUnits<LinearVelocity> left_Kv,
                         blazing::lyfast::KsUnits left_Ks,
                         blazing::lyfast::KvUnits<LinearVelocity> right_Kv,
                         blazing::lyfast::KsUnits right_Ks,
                         bool use_measured_voltage = false,
                         Time delta_time = 10_msec);

void angular_raw_ka_tuner(blazing::lyfast::KvUnits<LinearVelocity> left_Kv,
                          blazing::lyfast::KsUnits left_Ks,
                          blazing::lyfast::KvUnits<LinearVelocity> right_Kv,
                          blazing::lyfast::KsUnits right_Ks,
                          bool use_measured_voltage = false,
                          Time delta_time = 10_msec);
