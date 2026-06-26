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

#include <cstdint>

#include <array>
#include <limits>

#include "autodrive_d02_can/can_encoder.hpp"

namespace autodrive_d02_can
{
namespace
{

void ExpectFrameData(
  const VehicleControlCommand & command,
  const std::array<std::uint8_t, 8> & expected_data)
{
  CanFrameView frame{};
  ASSERT_EQ(VehicleControlEncodeResult::OK, EncodeVehicleControl(command, &frame));

  EXPECT_EQ(kVehicleControlCanId, frame.id);
  EXPECT_FALSE(frame.is_extended);
  EXPECT_FALSE(frame.is_rtr);
  EXPECT_FALSE(frame.is_error);
  EXPECT_EQ(8U, frame.dlc);
  EXPECT_EQ(expected_data, frame.data);
}

TEST(D02VehicleControlEncoder, encodesProtocolExamples)
{
  VehicleControlCommand brake{};
  brake.drive_mode = kDriveModeAuto;
  brake.shift_level = kShiftLevelNone;
  brake.brake_enable = true;
  ExpectFrameData(brake, {0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x80U});

  VehicleControlCommand motor_reverse{};
  motor_reverse.drive_mode = kDriveModeAuto;
  motor_reverse.shift_level = kShiftLevelNone;
  motor_reverse.left_motor_speed_rpm = -12.0F;
  motor_reverse.right_motor_speed_rpm = -12.0F;
  motor_reverse.brake_enable = false;
  ExpectFrameData(
    motor_reverse, {0x40U, 0x88U, 0xFFU, 0x88U, 0xFFU, 0x00U, 0x00U, 0x00U});

  VehicleControlCommand motor_forward{};
  motor_forward.drive_mode = kDriveModeAuto;
  motor_forward.shift_level = kShiftLevelNone;
  motor_forward.left_motor_speed_rpm = 12.0F;
  motor_forward.right_motor_speed_rpm = 12.0F;
  motor_forward.brake_enable = false;
  ExpectFrameData(
    motor_forward, {0x40U, 0x78U, 0x00U, 0x78U, 0x00U, 0x00U, 0x00U, 0x00U});

  VehicleControlCommand forward{};
  forward.drive_mode = kDriveModeAuto;
  forward.shift_level = kShiftLevelD;
  forward.left_motor_speed_rpm = 3.0F;
  forward.right_motor_speed_rpm = 3.0F;
  forward.brake_enable = false;
  ExpectFrameData(forward, {0x41U, 0x1EU, 0x00U, 0x1EU, 0x00U, 0x00U, 0x00U, 0x00U});

  VehicleControlCommand reverse{};
  reverse.drive_mode = kDriveModeAuto;
  reverse.shift_level = kShiftLevelR;
  reverse.left_motor_speed_rpm = 3.0F;
  reverse.right_motor_speed_rpm = 3.0F;
  reverse.brake_enable = false;
  ExpectFrameData(reverse, {0x43U, 0x1EU, 0x00U, 0x1EU, 0x00U, 0x00U, 0x00U, 0x00U});
}

TEST(D02VehicleControlEncoder, encodesExtendedControlFrameBits)
{
  VehicleControlCommand command{};
  command.drive_mode = kDriveModeAuto;
  command.shift_level = kShiftLevelD;
  command.left_motor_speed_rpm = 123.4F;
  command.right_motor_speed_rpm = -123.4F;
  command.brake_enable = true;
  command.left_turn_light_req = 1U;
  command.right_turn_light_req = 2U;
  command.position_light_req = 3U;
  command.low_beam_req = 1U;
  command.steering_angle_speed_valid = 1;
  command.brake_force_command_valid = 1U;

  CanFrameView frame{};
  ASSERT_EQ(VehicleControlEncodeResult::OK, EncodeVehicleControl(command, &frame));

  EXPECT_EQ(kVehicleControlCanId, frame.id);
  EXPECT_FALSE(frame.is_extended);
  EXPECT_FALSE(frame.is_rtr);
  EXPECT_FALSE(frame.is_error);
  EXPECT_EQ(8U, frame.dlc);
  EXPECT_EQ(0x41U, frame.data[0]);
  EXPECT_EQ(0xD2U, frame.data[1]);
  EXPECT_EQ(0x04U, frame.data[2]);
  EXPECT_EQ(0x2EU, frame.data[3]);
  EXPECT_EQ(0xFBU, frame.data[4]);
  EXPECT_EQ(0xC9U, frame.data[5]);
  EXPECT_EQ(0x01U, frame.data[6]);
  EXPECT_EQ(0xB0U, frame.data[7]);
}

TEST(D02VehicleControlEncoder, clampsMotorSpeedToProtocolRange)
{
  VehicleControlCommand command{};
  command.drive_mode = kDriveModeAuto;
  command.shift_level = kShiftLevelD;
  command.left_motor_speed_rpm = 800.0F;
  command.right_motor_speed_rpm = -800.0F;
  command.brake_enable = false;

  CanFrameView frame{};
  ASSERT_EQ(VehicleControlEncodeResult::OK, EncodeVehicleControl(command, &frame));

  EXPECT_EQ(0xA0U, frame.data[1]);
  EXPECT_EQ(0x0FU, frame.data[2]);
  EXPECT_EQ(0x60U, frame.data[3]);
  EXPECT_EQ(0xF0U, frame.data[4]);
}

TEST(D02VehicleControlEncoder, encodesSafeCommand)
{
  CanFrameView frame{};
  ASSERT_EQ(
    VehicleControlEncodeResult::OK,
    EncodeVehicleControl(MakeSafeVehicleControlCommand(), &frame));

  EXPECT_EQ(0xC2U, frame.data[0]);
  EXPECT_EQ(0x00U, frame.data[1]);
  EXPECT_EQ(0x00U, frame.data[2]);
  EXPECT_EQ(0x00U, frame.data[3]);
  EXPECT_EQ(0x00U, frame.data[4]);
  EXPECT_EQ(0x00U, frame.data[5]);
  EXPECT_EQ(0x00U, frame.data[6]);
  EXPECT_EQ(0x80U, frame.data[7]);
}

TEST(D02VehicleControlEncoder, rejectsInvalidCommands)
{
  VehicleControlCommand command{};
  command.drive_mode = kDriveModeAuto;
  command.shift_level = kShiftLevelD;

  CanFrameView frame{};
  EXPECT_EQ(
    VehicleControlEncodeResult::INVALID_OUTPUT,
    EncodeVehicleControl(command, nullptr));

  command.drive_mode = 0U;
  EXPECT_EQ(
    VehicleControlEncodeResult::INVALID_DRIVE_MODE,
    EncodeVehicleControl(command, &frame));

  command.drive_mode = kDriveModeAuto;
  command.shift_level = 0U;
  EXPECT_EQ(VehicleControlEncodeResult::OK, EncodeVehicleControl(command, &frame));

  command.shift_level = 4U;
  EXPECT_EQ(
    VehicleControlEncodeResult::INVALID_SHIFT_LEVEL,
    EncodeVehicleControl(command, &frame));

  command.shift_level = kShiftLevelD;
  command.left_motor_speed_rpm = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(
    VehicleControlEncodeResult::INVALID_MOTOR_SPEED,
    EncodeVehicleControl(command, &frame));

  command.left_motor_speed_rpm = 0.0F;
  command.right_motor_speed_rpm = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(
    VehicleControlEncodeResult::INVALID_MOTOR_SPEED,
    EncodeVehicleControl(command, &frame));

  command.right_motor_speed_rpm = 0.0F;
  command.low_beam_req = 4U;
  EXPECT_EQ(
    VehicleControlEncodeResult::INVALID_TWO_BIT_FIELD,
    EncodeVehicleControl(command, &frame));

  command.low_beam_req = 0U;
  command.steering_angle_speed_valid = 2;
  EXPECT_EQ(
    VehicleControlEncodeResult::INVALID_STEERING_ANGLE_SPEED_VALID,
    EncodeVehicleControl(command, &frame));

  command.steering_angle_speed_valid = 0;
  command.brake_force_command_valid = 2U;
  EXPECT_EQ(
    VehicleControlEncodeResult::INVALID_BRAKE_FORCE_COMMAND_VALID,
    EncodeVehicleControl(command, &frame));
}

}  // namespace
}  // namespace autodrive_d02_can
