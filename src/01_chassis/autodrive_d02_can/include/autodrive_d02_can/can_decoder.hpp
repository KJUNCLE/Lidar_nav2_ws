// Copyright 2026 KJUNCLE
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTODRIVE_D02_CAN__CAN_DECODER_HPP_
#define AUTODRIVE_D02_CAN__CAN_DECODER_HPP_

#include <cstdint>

#include "autodrive_d02_can/can_encoder.hpp"

namespace autodrive_d02_can
{

constexpr std::uint32_t kTargetSpeedFeedbackCanId = 0x7F1U;
constexpr std::uint32_t kWheelSpeedFeedbackCanId = 0x168U;
constexpr std::uint32_t kBatteryFeedbackCanId = 0x100U;
constexpr std::uint32_t kVehicleStatusFeedbackCanId = 0x051U;
constexpr std::uint32_t kWarningLevelFeedbackCanId = 0x077U;
constexpr float kTargetSpeedFeedbackFactor = 0.1F;
constexpr float kWheelSpeedFeedbackFactor = 0.1F;
constexpr float kBatteryVoltageFactor = 0.1F;
constexpr float kBatteryCurrentFactor = 0.1F;
constexpr float kBatterySocFactor = 1.0F;
constexpr float kVehicleStatusSteeringAngleFactor = 0.1F;
constexpr float kVehicleStatusSpeedFactor = 0.1F;

struct TargetSpeedFeedback
{
  std::uint32_t can_id{};
  std::int16_t hardware_target_speed_raw{};
  float hardware_target_speed_kmh{};
  std::int16_t scu_target_speed_feedback_raw{};
  float scu_target_speed_feedback_kmh{};
  std::int16_t vehicle_target_speed_raw{};
  float vehicle_target_speed_kmh{};
  std::int16_t vehicle_target_speed_rpm_raw{};
  float vehicle_target_speed_rpm{};
};

struct WheelSpeedFeedback
{
  std::uint32_t can_id{};
  std::int16_t front_left_rpm_raw{};
  float front_left_rpm{};
  std::int16_t front_right_rpm_raw{};
  float front_right_rpm{};
  std::int16_t rear_left_rpm_raw{};
  float rear_left_rpm{};
  std::int16_t rear_right_rpm_raw{};
  float rear_right_rpm{};
};

struct BatteryFeedback
{
  std::uint32_t can_id{};
  std::uint16_t bms_voltage_raw{};
  float bms_voltage_v{};
  std::int16_t bms_current_raw{};
  float bms_current_a{};
  std::uint16_t bms_soc_raw{};
  float bms_soc_percent{};
};

struct VehicleStatusFeedback
{
  std::uint32_t can_id{};
  std::uint8_t ccu_shift_level_status{};
  std::uint8_t ccu_parking_status{};
  std::uint8_t ccu_ignition_status{};
  std::uint8_t ccu_drive_mode_shift_button{};
  std::uint8_t steering_wheel_direction{};
  std::uint16_t ccu_steering_wheel_angle_raw{};
  float ccu_steering_wheel_angle_deg{};
  std::uint16_t ccu_vehicle_speed_raw{};
  float ccu_vehicle_speed_kmh{};
  std::uint8_t ccu_drive_mode{};
  std::uint8_t remote_brake_request_status{};
  std::uint8_t emergency_brake_request_status{};
  std::uint8_t scu_brake_signal_status{};
  std::uint8_t touch_brake_request_status{};
  std::uint8_t handle_brake_request_status{};
  std::uint8_t handle_mode_flag_status{};
  std::uint8_t left_turn_light_status{};
  std::uint8_t right_turn_light_status{};
  std::uint8_t position_light_status{};
  std::uint8_t low_beam_status{};
};

struct WarningLevelFeedback
{
  std::uint32_t can_id{};
  std::uint8_t bms_soc_warning{};
  std::uint8_t mcu_disconnect_warning{};
  std::uint8_t mcu_motor_warning{};
  std::uint8_t mcu_overspeed_warning{};
  std::uint8_t turn_disconnect_warning{};
  std::uint8_t turn_lock_warning{};
  std::uint8_t turn_unstoppable_warning{};
  std::uint8_t sas_angle_error{};
  std::uint8_t brake_error_warning{};
};

bool DecodeTargetSpeedFeedback(
  const CanFrameView & frame,
  TargetSpeedFeedback * feedback);

bool DecodeWheelSpeedFeedback(
  const CanFrameView & frame,
  WheelSpeedFeedback * feedback);

bool DecodeBatteryFeedback(
  const CanFrameView & frame,
  BatteryFeedback * feedback);

bool DecodeVehicleStatusFeedback(
  const CanFrameView & frame,
  VehicleStatusFeedback * feedback);

bool DecodeWarningLevelFeedback(
  const CanFrameView & frame,
  WarningLevelFeedback * feedback);

}  // namespace autodrive_d02_can

#endif  // AUTODRIVE_D02_CAN__CAN_DECODER_HPP_
