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

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "autodrive_d02_can/can_decoder.hpp"

namespace autodrive_d02_can
{
namespace
{

void EncodeLittleEndianUInt16(
  const std::uint16_t value,
  const std::size_t offset,
  std::array<std::uint8_t, 8> * data)
{
  (*data)[offset] = static_cast<std::uint8_t>(value & 0xFFU);
  (*data)[offset + 1U] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
}

void EncodeLittleEndianInt16(
  const std::int16_t value,
  const std::size_t offset,
  std::array<std::uint8_t, 8> * data)
{
  EncodeLittleEndianUInt16(static_cast<std::uint16_t>(value), offset, data);
}

void SetUnsignedBits(
  std::array<std::uint8_t, 8> * data,
  const std::size_t start_bit,
  const std::size_t bit_length,
  const std::uint32_t value)
{
  for (std::size_t bit_index = 0U; bit_index < bit_length; ++bit_index) {
    const std::size_t target_bit = start_bit + bit_index;
    const std::uint8_t bit_value =
      static_cast<std::uint8_t>((value >> bit_index) & 0x1U);
    if (bit_value != 0U) {
      (*data)[target_bit / 8U] =
        static_cast<std::uint8_t>((*data)[target_bit / 8U] | (1U << (target_bit % 8U)));
    }
  }
}

CanFrameView MakeFrame(const std::uint32_t can_id)
{
  CanFrameView frame{};
  frame.id = can_id;
  frame.dlc = 8U;
  return frame;
}

CanFrameView MakeTargetSpeedFrame(
  const std::int16_t hardware_target_speed_raw,
  const std::int16_t scu_target_speed_feedback_raw,
  const std::int16_t vehicle_target_speed_raw,
  const std::int16_t vehicle_target_speed_rpm_raw)
{
  auto frame = MakeFrame(kTargetSpeedFeedbackCanId);
  EncodeLittleEndianInt16(hardware_target_speed_raw, 0U, &frame.data);
  EncodeLittleEndianInt16(scu_target_speed_feedback_raw, 2U, &frame.data);
  EncodeLittleEndianInt16(vehicle_target_speed_raw, 4U, &frame.data);
  EncodeLittleEndianInt16(vehicle_target_speed_rpm_raw, 6U, &frame.data);
  return frame;
}

TEST(D02CanDecoder, decodesTargetSpeedFeedback)
{
  const auto frame = MakeTargetSpeedFrame(12, -30, 345, -1234);

  TargetSpeedFeedback feedback{};
  ASSERT_TRUE(DecodeTargetSpeedFeedback(frame, &feedback));

  EXPECT_EQ(kTargetSpeedFeedbackCanId, feedback.can_id);
  EXPECT_EQ(12, feedback.hardware_target_speed_raw);
  EXPECT_FLOAT_EQ(1.2F, feedback.hardware_target_speed_kmh);
  EXPECT_EQ(-30, feedback.scu_target_speed_feedback_raw);
  EXPECT_FLOAT_EQ(-3.0F, feedback.scu_target_speed_feedback_kmh);
  EXPECT_EQ(345, feedback.vehicle_target_speed_raw);
  EXPECT_FLOAT_EQ(34.5F, feedback.vehicle_target_speed_kmh);
  EXPECT_EQ(-1234, feedback.vehicle_target_speed_rpm_raw);
  EXPECT_FLOAT_EQ(-123.4F, feedback.vehicle_target_speed_rpm);
}

TEST(D02CanDecoder, decodesWheelSpeedFeedback)
{
  auto frame = MakeFrame(kWheelSpeedFeedbackCanId);
  EncodeLittleEndianInt16(10, 0U, &frame.data);
  EncodeLittleEndianInt16(-20, 2U, &frame.data);
  EncodeLittleEndianInt16(345, 4U, &frame.data);
  EncodeLittleEndianInt16(-1234, 6U, &frame.data);

  WheelSpeedFeedback feedback{};
  ASSERT_TRUE(DecodeWheelSpeedFeedback(frame, &feedback));

  EXPECT_EQ(kWheelSpeedFeedbackCanId, feedback.can_id);
  EXPECT_EQ(10, feedback.front_left_rpm_raw);
  EXPECT_FLOAT_EQ(1.0F, feedback.front_left_rpm);
  EXPECT_EQ(-20, feedback.front_right_rpm_raw);
  EXPECT_FLOAT_EQ(-2.0F, feedback.front_right_rpm);
  EXPECT_EQ(345, feedback.rear_left_rpm_raw);
  EXPECT_FLOAT_EQ(34.5F, feedback.rear_left_rpm);
  EXPECT_EQ(-1234, feedback.rear_right_rpm_raw);
  EXPECT_FLOAT_EQ(-123.4F, feedback.rear_right_rpm);
}

TEST(D02CanDecoder, decodesBatteryFeedback)
{
  auto frame = MakeFrame(kBatteryFeedbackCanId);
  frame.dlc = 6U;
  EncodeLittleEndianUInt16(1234U, 0U, &frame.data);
  EncodeLittleEndianInt16(-567, 2U, &frame.data);
  EncodeLittleEndianUInt16(88U, 4U, &frame.data);

  BatteryFeedback feedback{};
  ASSERT_TRUE(DecodeBatteryFeedback(frame, &feedback));

  EXPECT_EQ(kBatteryFeedbackCanId, feedback.can_id);
  EXPECT_EQ(1234U, feedback.bms_voltage_raw);
  EXPECT_FLOAT_EQ(123.4F, feedback.bms_voltage_v);
  EXPECT_EQ(-567, feedback.bms_current_raw);
  EXPECT_FLOAT_EQ(-56.7F, feedback.bms_current_a);
  EXPECT_EQ(88U, feedback.bms_soc_raw);
  EXPECT_FLOAT_EQ(88.0F, feedback.bms_soc_percent);
}

TEST(D02CanDecoder, decodesVehicleStatusFeedback)
{
  auto frame = MakeFrame(kVehicleStatusFeedbackCanId);
  SetUnsignedBits(&frame.data, 0U, 2U, 3U);
  SetUnsignedBits(&frame.data, 2U, 1U, 1U);
  SetUnsignedBits(&frame.data, 3U, 2U, 2U);
  SetUnsignedBits(&frame.data, 5U, 1U, 1U);
  SetUnsignedBits(&frame.data, 7U, 1U, 1U);
  SetUnsignedBits(&frame.data, 8U, 12U, 1200U);
  SetUnsignedBits(&frame.data, 20U, 9U, 321U);
  SetUnsignedBits(&frame.data, 29U, 3U, 5U);
  SetUnsignedBits(&frame.data, 32U, 1U, 1U);
  SetUnsignedBits(&frame.data, 33U, 1U, 0U);
  SetUnsignedBits(&frame.data, 34U, 1U, 1U);
  SetUnsignedBits(&frame.data, 35U, 1U, 1U);
  SetUnsignedBits(&frame.data, 36U, 1U, 0U);
  SetUnsignedBits(&frame.data, 37U, 1U, 1U);
  SetUnsignedBits(&frame.data, 56U, 1U, 1U);
  SetUnsignedBits(&frame.data, 57U, 1U, 1U);
  SetUnsignedBits(&frame.data, 59U, 1U, 1U);
  SetUnsignedBits(&frame.data, 60U, 1U, 0U);

  VehicleStatusFeedback feedback{};
  ASSERT_TRUE(DecodeVehicleStatusFeedback(frame, &feedback));

  EXPECT_EQ(kVehicleStatusFeedbackCanId, feedback.can_id);
  EXPECT_EQ(3U, feedback.ccu_shift_level_status);
  EXPECT_EQ(1U, feedback.ccu_parking_status);
  EXPECT_EQ(2U, feedback.ccu_ignition_status);
  EXPECT_EQ(1U, feedback.ccu_drive_mode_shift_button);
  EXPECT_EQ(1U, feedback.steering_wheel_direction);
  EXPECT_EQ(1200U, feedback.ccu_steering_wheel_angle_raw);
  EXPECT_FLOAT_EQ(120.0F, feedback.ccu_steering_wheel_angle_deg);
  EXPECT_EQ(321U, feedback.ccu_vehicle_speed_raw);
  EXPECT_FLOAT_EQ(32.1F, feedback.ccu_vehicle_speed_kmh);
  EXPECT_EQ(5U, feedback.ccu_drive_mode);
  EXPECT_EQ(1U, feedback.remote_brake_request_status);
  EXPECT_EQ(0U, feedback.emergency_brake_request_status);
  EXPECT_EQ(1U, feedback.scu_brake_signal_status);
  EXPECT_EQ(1U, feedback.touch_brake_request_status);
  EXPECT_EQ(0U, feedback.handle_brake_request_status);
  EXPECT_EQ(1U, feedback.handle_mode_flag_status);
  EXPECT_EQ(1U, feedback.left_turn_light_status);
  EXPECT_EQ(1U, feedback.right_turn_light_status);
  EXPECT_EQ(1U, feedback.position_light_status);
  EXPECT_EQ(0U, feedback.low_beam_status);
}

TEST(D02CanDecoder, decodesWarningLevelFeedback)
{
  auto frame = MakeFrame(kWarningLevelFeedbackCanId);
  frame.dlc = 4U;
  SetUnsignedBits(&frame.data, 0U, 3U, 0U);
  SetUnsignedBits(&frame.data, 3U, 3U, 1U);
  SetUnsignedBits(&frame.data, 6U, 3U, 2U);
  SetUnsignedBits(&frame.data, 9U, 3U, 3U);
  SetUnsignedBits(&frame.data, 12U, 3U, 4U);
  SetUnsignedBits(&frame.data, 15U, 3U, 5U);
  SetUnsignedBits(&frame.data, 18U, 3U, 6U);
  SetUnsignedBits(&frame.data, 21U, 3U, 7U);
  SetUnsignedBits(&frame.data, 24U, 3U, 1U);

  WarningLevelFeedback feedback{};
  ASSERT_TRUE(DecodeWarningLevelFeedback(frame, &feedback));

  EXPECT_EQ(kWarningLevelFeedbackCanId, feedback.can_id);
  EXPECT_EQ(0U, feedback.bms_soc_warning);
  EXPECT_EQ(1U, feedback.mcu_disconnect_warning);
  EXPECT_EQ(2U, feedback.mcu_motor_warning);
  EXPECT_EQ(3U, feedback.mcu_overspeed_warning);
  EXPECT_EQ(4U, feedback.turn_disconnect_warning);
  EXPECT_EQ(5U, feedback.turn_lock_warning);
  EXPECT_EQ(6U, feedback.turn_unstoppable_warning);
  EXPECT_EQ(7U, feedback.sas_angle_error);
  EXPECT_EQ(1U, feedback.brake_error_warning);
}

TEST(D02CanDecoder, ignoresInvalidTargetSpeedFrames)
{
  TargetSpeedFeedback feedback{};

  auto frame = MakeTargetSpeedFrame(1, 2, 3, 4);
  frame.id = 0x120U;
  EXPECT_FALSE(DecodeTargetSpeedFeedback(frame, &feedback));

  frame = MakeTargetSpeedFrame(1, 2, 3, 4);
  frame.is_extended = true;
  EXPECT_FALSE(DecodeTargetSpeedFeedback(frame, &feedback));

  frame = MakeTargetSpeedFrame(1, 2, 3, 4);
  frame.is_rtr = true;
  EXPECT_FALSE(DecodeTargetSpeedFeedback(frame, &feedback));

  frame = MakeTargetSpeedFrame(1, 2, 3, 4);
  frame.is_error = true;
  EXPECT_FALSE(DecodeTargetSpeedFeedback(frame, &feedback));

  frame = MakeTargetSpeedFrame(1, 2, 3, 4);
  frame.dlc = 7U;
  EXPECT_FALSE(DecodeTargetSpeedFeedback(frame, &feedback));

  frame = MakeTargetSpeedFrame(1, 2, 3, 4);
  EXPECT_FALSE(DecodeTargetSpeedFeedback(frame, nullptr));
}

TEST(D02CanDecoder, ignoresShortOrMismatchedFramesForNewFeedback)
{
  WheelSpeedFeedback wheel_speed{};
  auto wheel_frame = MakeFrame(kWheelSpeedFeedbackCanId);
  wheel_frame.dlc = 7U;
  EXPECT_FALSE(DecodeWheelSpeedFeedback(wheel_frame, &wheel_speed));

  BatteryFeedback battery{};
  auto battery_frame = MakeFrame(kBatteryFeedbackCanId);
  battery_frame.dlc = 5U;
  EXPECT_FALSE(DecodeBatteryFeedback(battery_frame, &battery));

  battery_frame = MakeFrame(0x101U);
  EXPECT_FALSE(DecodeBatteryFeedback(battery_frame, &battery));

  VehicleStatusFeedback vehicle_status{};
  auto status_frame = MakeFrame(kVehicleStatusFeedbackCanId);
  status_frame.dlc = 7U;
  EXPECT_FALSE(DecodeVehicleStatusFeedback(status_frame, &vehicle_status));

  status_frame = MakeFrame(kVehicleStatusFeedbackCanId);
  status_frame.is_extended = true;
  EXPECT_FALSE(DecodeVehicleStatusFeedback(status_frame, &vehicle_status));

  status_frame = MakeFrame(kVehicleStatusFeedbackCanId);
  status_frame.is_rtr = true;
  EXPECT_FALSE(DecodeVehicleStatusFeedback(status_frame, &vehicle_status));

  status_frame = MakeFrame(kVehicleStatusFeedbackCanId);
  status_frame.is_error = true;
  EXPECT_FALSE(DecodeVehicleStatusFeedback(status_frame, &vehicle_status));

  status_frame = MakeFrame(kWheelSpeedFeedbackCanId);
  EXPECT_FALSE(DecodeVehicleStatusFeedback(status_frame, &vehicle_status));
  EXPECT_FALSE(DecodeWheelSpeedFeedback(MakeFrame(kVehicleStatusFeedbackCanId), nullptr));
  EXPECT_FALSE(DecodeBatteryFeedback(MakeFrame(kBatteryFeedbackCanId), nullptr));

  WarningLevelFeedback warning_level{};
  auto warning_frame = MakeFrame(kWarningLevelFeedbackCanId);
  warning_frame.dlc = 3U;
  EXPECT_FALSE(DecodeWarningLevelFeedback(warning_frame, &warning_level));

  warning_frame = MakeFrame(kWarningLevelFeedbackCanId);
  warning_frame.is_error = true;
  EXPECT_FALSE(DecodeWarningLevelFeedback(warning_frame, &warning_level));

  warning_frame = MakeFrame(kVehicleStatusFeedbackCanId);
  EXPECT_FALSE(DecodeWarningLevelFeedback(warning_frame, &warning_level));
  EXPECT_FALSE(DecodeWarningLevelFeedback(MakeFrame(kWarningLevelFeedbackCanId), nullptr));
}

}  // namespace
}  // namespace autodrive_d02_can
