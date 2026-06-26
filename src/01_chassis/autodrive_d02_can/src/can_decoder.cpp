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

#include "autodrive_d02_can/can_decoder.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace autodrive_d02_can
{
namespace
{

std::uint16_t DecodeLittleEndianUInt16(
  const std::array<std::uint8_t, 8> & data,
  const std::size_t offset)
{
  return static_cast<std::uint16_t>(
    static_cast<std::uint16_t>(data[offset]) |
    static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[offset + 1U]) << 8U));
}

std::int16_t DecodeLittleEndianInt16(
  const std::array<std::uint8_t, 8> & data,
  const std::size_t offset)
{
  const auto raw = DecodeLittleEndianUInt16(data, offset);
  if (raw <= 0x7FFFU) {
    return static_cast<std::int16_t>(raw);
  }

  return static_cast<std::int16_t>(static_cast<int>(raw) - 0x10000);
}

std::uint32_t DecodeLittleEndianUnsignedBits(
  const std::array<std::uint8_t, 8> & data,
  const std::size_t start_bit,
  const std::size_t bit_length)
{
  std::uint32_t value = 0U;
  for (std::size_t bit_index = 0U; bit_index < bit_length; ++bit_index) {
    const std::size_t source_bit = start_bit + bit_index;
    const auto bit_value = static_cast<std::uint32_t>(
      (data[source_bit / 8U] >> (source_bit % 8U)) & 0x1U);
    value |= bit_value << bit_index;
  }
  return value;
}

bool IsValidFrameForDecode(
  const CanFrameView & frame,
  const std::uint32_t can_id,
  const std::uint8_t minimum_dlc)
{
  return !frame.is_extended && !frame.is_rtr && !frame.is_error &&
         frame.dlc >= minimum_dlc && frame.id == can_id;
}

float ScaleInt16(const std::int16_t raw, const float factor)
{
  return static_cast<float>(raw) * factor;
}

float ScaleUInt16(const std::uint16_t raw, const float factor)
{
  return static_cast<float>(raw) * factor;
}

}  // namespace

bool DecodeTargetSpeedFeedback(
  const CanFrameView & frame,
  TargetSpeedFeedback * feedback)
{
  if (feedback == nullptr ||
    !IsValidFrameForDecode(frame, kTargetSpeedFeedbackCanId, 8U))
  {
    return false;
  }

  TargetSpeedFeedback decoded{};
  decoded.can_id = frame.id;
  decoded.hardware_target_speed_raw = DecodeLittleEndianInt16(frame.data, 0U);
  decoded.scu_target_speed_feedback_raw = DecodeLittleEndianInt16(frame.data, 2U);
  decoded.vehicle_target_speed_raw = DecodeLittleEndianInt16(frame.data, 4U);
  decoded.vehicle_target_speed_rpm_raw = DecodeLittleEndianInt16(frame.data, 6U);
  decoded.hardware_target_speed_kmh =
    ScaleInt16(decoded.hardware_target_speed_raw, kTargetSpeedFeedbackFactor);
  decoded.scu_target_speed_feedback_kmh =
    ScaleInt16(decoded.scu_target_speed_feedback_raw, kTargetSpeedFeedbackFactor);
  decoded.vehicle_target_speed_kmh =
    ScaleInt16(decoded.vehicle_target_speed_raw, kTargetSpeedFeedbackFactor);
  decoded.vehicle_target_speed_rpm =
    ScaleInt16(decoded.vehicle_target_speed_rpm_raw, kTargetSpeedFeedbackFactor);

  *feedback = decoded;
  return true;
}

bool DecodeWheelSpeedFeedback(
  const CanFrameView & frame,
  WheelSpeedFeedback * feedback)
{
  if (feedback == nullptr ||
    !IsValidFrameForDecode(frame, kWheelSpeedFeedbackCanId, 8U))
  {
    return false;
  }

  WheelSpeedFeedback decoded{};
  decoded.can_id = frame.id;
  decoded.front_left_rpm_raw = DecodeLittleEndianInt16(frame.data, 0U);
  decoded.front_right_rpm_raw = DecodeLittleEndianInt16(frame.data, 2U);
  decoded.rear_left_rpm_raw = DecodeLittleEndianInt16(frame.data, 4U);
  decoded.rear_right_rpm_raw = DecodeLittleEndianInt16(frame.data, 6U);
  decoded.front_left_rpm = ScaleInt16(decoded.front_left_rpm_raw, kWheelSpeedFeedbackFactor);
  decoded.front_right_rpm = ScaleInt16(decoded.front_right_rpm_raw, kWheelSpeedFeedbackFactor);
  decoded.rear_left_rpm = ScaleInt16(decoded.rear_left_rpm_raw, kWheelSpeedFeedbackFactor);
  decoded.rear_right_rpm = ScaleInt16(decoded.rear_right_rpm_raw, kWheelSpeedFeedbackFactor);

  *feedback = decoded;
  return true;
}

bool DecodeBatteryFeedback(
  const CanFrameView & frame,
  BatteryFeedback * feedback)
{
  if (feedback == nullptr ||
    !IsValidFrameForDecode(frame, kBatteryFeedbackCanId, 6U))
  {
    return false;
  }

  BatteryFeedback decoded{};
  decoded.can_id = frame.id;
  decoded.bms_voltage_raw = DecodeLittleEndianUInt16(frame.data, 0U);
  decoded.bms_current_raw = DecodeLittleEndianInt16(frame.data, 2U);
  decoded.bms_soc_raw = DecodeLittleEndianUInt16(frame.data, 4U);
  decoded.bms_voltage_v = ScaleUInt16(decoded.bms_voltage_raw, kBatteryVoltageFactor);
  decoded.bms_current_a = ScaleInt16(decoded.bms_current_raw, kBatteryCurrentFactor);
  decoded.bms_soc_percent = ScaleUInt16(decoded.bms_soc_raw, kBatterySocFactor);

  *feedback = decoded;
  return true;
}

bool DecodeVehicleStatusFeedback(
  const CanFrameView & frame,
  VehicleStatusFeedback * feedback)
{
  if (feedback == nullptr ||
    !IsValidFrameForDecode(frame, kVehicleStatusFeedbackCanId, 8U))
  {
    return false;
  }

  VehicleStatusFeedback decoded{};
  decoded.can_id = frame.id;
  decoded.ccu_shift_level_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 0U, 2U));
  decoded.ccu_parking_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 2U, 1U));
  decoded.ccu_ignition_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 3U, 2U));
  decoded.ccu_drive_mode_shift_button =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 5U, 1U));
  decoded.steering_wheel_direction =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 7U, 1U));
  decoded.ccu_steering_wheel_angle_raw =
    static_cast<std::uint16_t>(DecodeLittleEndianUnsignedBits(frame.data, 8U, 12U));
  decoded.ccu_steering_wheel_angle_deg =
    ScaleUInt16(decoded.ccu_steering_wheel_angle_raw, kVehicleStatusSteeringAngleFactor);
  decoded.ccu_vehicle_speed_raw =
    static_cast<std::uint16_t>(DecodeLittleEndianUnsignedBits(frame.data, 20U, 9U));
  decoded.ccu_vehicle_speed_kmh =
    ScaleUInt16(decoded.ccu_vehicle_speed_raw, kVehicleStatusSpeedFactor);
  decoded.ccu_drive_mode =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 29U, 3U));
  decoded.remote_brake_request_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 32U, 1U));
  decoded.emergency_brake_request_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 33U, 1U));
  decoded.scu_brake_signal_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 34U, 1U));
  decoded.touch_brake_request_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 35U, 1U));
  decoded.handle_brake_request_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 36U, 1U));
  decoded.handle_mode_flag_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 37U, 1U));
  decoded.left_turn_light_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 56U, 1U));
  decoded.right_turn_light_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 57U, 1U));
  decoded.position_light_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 59U, 1U));
  decoded.low_beam_status =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 60U, 1U));

  *feedback = decoded;
  return true;
}

bool DecodeWarningLevelFeedback(
  const CanFrameView & frame,
  WarningLevelFeedback * feedback)
{
  if (feedback == nullptr ||
    !IsValidFrameForDecode(frame, kWarningLevelFeedbackCanId, 4U))
  {
    return false;
  }

  WarningLevelFeedback decoded{};
  decoded.can_id = frame.id;
  decoded.bms_soc_warning =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 0U, 3U));
  decoded.mcu_disconnect_warning =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 3U, 3U));
  decoded.mcu_motor_warning =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 6U, 3U));
  decoded.mcu_overspeed_warning =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 9U, 3U));
  decoded.turn_disconnect_warning =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 12U, 3U));
  decoded.turn_lock_warning =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 15U, 3U));
  decoded.turn_unstoppable_warning =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 18U, 3U));
  decoded.sas_angle_error =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 21U, 3U));
  decoded.brake_error_warning =
    static_cast<std::uint8_t>(DecodeLittleEndianUnsignedBits(frame.data, 24U, 3U));

  *feedback = decoded;
  return true;
}

}  // namespace autodrive_d02_can
