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

#include "autodrive_d02_can/can_encoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace autodrive_d02_can
{
namespace
{

bool IsValidDriveMode(const std::uint8_t drive_mode)
{
  return drive_mode == kDriveModeAuto || drive_mode == kDriveModeRemote;
}

bool IsValidShiftLevel(const std::uint8_t shift_level)
{
  return shift_level == kShiftLevelNone ||
         shift_level == kShiftLevelD ||
         shift_level == kShiftLevelN ||
         shift_level == kShiftLevelR;
}

bool IsValidTwoBitField(const std::uint8_t value)
{
  return value <= 3U;
}

bool IsValidOneBitSignedFlag(const std::int8_t value)
{
  return value == 0 || value == 1;
}

bool IsValidOneBitUnsignedFlag(const std::uint8_t value)
{
  return value <= 1U;
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

std::uint16_t MotorSpeedRpmToRaw(const float motor_speed_rpm)
{
  const float clamped = std::min(
    std::max(motor_speed_rpm, kMotorSpeedMinRpm),
    kMotorSpeedMaxRpm);
  const auto raw = static_cast<std::int16_t>(
    std::lround(clamped / kMotorSpeedFactorRpm));
  return static_cast<std::uint16_t>(raw);
}

}  // namespace

VehicleControlCommand MakeSafeVehicleControlCommand()
{
  VehicleControlCommand command{};
  command.drive_mode = kDriveModeRemote;
  command.shift_level = kShiftLevelN;
  command.left_motor_speed_rpm = 0.0F;
  command.right_motor_speed_rpm = 0.0F;
  command.brake_enable = true;
  return command;
}

VehicleControlEncodeResult EncodeVehicleControl(
  const VehicleControlCommand & command,
  CanFrameView * frame)
{
  if (frame == nullptr) {
    return VehicleControlEncodeResult::INVALID_OUTPUT;
  }

  if (!IsValidDriveMode(command.drive_mode)) {
    return VehicleControlEncodeResult::INVALID_DRIVE_MODE;
  }

  if (!IsValidShiftLevel(command.shift_level)) {
    return VehicleControlEncodeResult::INVALID_SHIFT_LEVEL;
  }

  if (!std::isfinite(command.left_motor_speed_rpm) ||
    !std::isfinite(command.right_motor_speed_rpm))
  {
    return VehicleControlEncodeResult::INVALID_MOTOR_SPEED;
  }

  if (!IsValidTwoBitField(command.left_turn_light_req) ||
    !IsValidTwoBitField(command.right_turn_light_req) ||
    !IsValidTwoBitField(command.position_light_req) ||
    !IsValidTwoBitField(command.low_beam_req))
  {
    return VehicleControlEncodeResult::INVALID_TWO_BIT_FIELD;
  }

  if (!IsValidOneBitSignedFlag(command.steering_angle_speed_valid)) {
    return VehicleControlEncodeResult::INVALID_STEERING_ANGLE_SPEED_VALID;
  }

  if (!IsValidOneBitUnsignedFlag(command.brake_force_command_valid)) {
    return VehicleControlEncodeResult::INVALID_BRAKE_FORCE_COMMAND_VALID;
  }

  CanFrameView encoded{};
  encoded.id = kVehicleControlCanId;
  encoded.is_extended = false;
  encoded.is_rtr = false;
  encoded.is_error = false;
  encoded.dlc = 8U;

  SetUnsignedBits(&encoded.data, 0U, 2U, command.shift_level);
  SetUnsignedBits(&encoded.data, 6U, 2U, command.drive_mode);
  SetUnsignedBits(
    &encoded.data, 8U, 16U,
    MotorSpeedRpmToRaw(command.left_motor_speed_rpm));
  SetUnsignedBits(
    &encoded.data, 24U, 16U,
    MotorSpeedRpmToRaw(command.right_motor_speed_rpm));
  SetUnsignedBits(&encoded.data, 40U, 2U, command.left_turn_light_req);
  SetUnsignedBits(&encoded.data, 42U, 2U, command.right_turn_light_req);
  SetUnsignedBits(&encoded.data, 46U, 2U, command.position_light_req);
  SetUnsignedBits(&encoded.data, 48U, 2U, command.low_beam_req);
  SetUnsignedBits(
    &encoded.data, 60U, 1U,
    static_cast<std::uint32_t>(command.steering_angle_speed_valid));
  SetUnsignedBits(&encoded.data, 61U, 1U, command.brake_force_command_valid);
  SetUnsignedBits(&encoded.data, 63U, 1U, command.brake_enable ? 1U : 0U);

  *frame = encoded;
  return VehicleControlEncodeResult::OK;
}

const char * VehicleControlEncodeResultToString(const VehicleControlEncodeResult result)
{
  switch (result) {
    case VehicleControlEncodeResult::OK:
      return "ok";
    case VehicleControlEncodeResult::INVALID_OUTPUT:
      return "invalid output";
    case VehicleControlEncodeResult::INVALID_DRIVE_MODE:
      return "invalid drive mode";
    case VehicleControlEncodeResult::INVALID_SHIFT_LEVEL:
      return "invalid shift level";
    case VehicleControlEncodeResult::INVALID_MOTOR_SPEED:
      return "invalid motor speed";
    case VehicleControlEncodeResult::INVALID_TWO_BIT_FIELD:
      return "invalid two-bit field";
    case VehicleControlEncodeResult::INVALID_STEERING_ANGLE_SPEED_VALID:
      return "invalid steering angle speed valid";
    case VehicleControlEncodeResult::INVALID_BRAKE_FORCE_COMMAND_VALID:
      return "invalid brake force command valid";
    default:
      return "unknown encode result";
  }
}

}  // namespace autodrive_d02_can
