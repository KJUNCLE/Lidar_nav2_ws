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

#ifndef AUTODRIVE_D02_CAN__CAN_ENCODER_HPP_
#define AUTODRIVE_D02_CAN__CAN_ENCODER_HPP_

#include <array>
#include <cstdint>

namespace autodrive_d02_can
{

struct CanFrameView
{
  std::uint32_t id{};
  bool is_extended{};
  bool is_rtr{};
  bool is_error{};
  std::uint8_t dlc{};
  std::array<std::uint8_t, 8> data{};
};

constexpr std::uint32_t kVehicleControlCanId = 0x122U;
constexpr std::uint8_t kDriveModeAuto = 1U;
constexpr std::uint8_t kDriveModeRemote = 3U;
constexpr std::uint8_t kShiftLevelNone = 0U;
constexpr std::uint8_t kShiftLevelD = 1U;
constexpr std::uint8_t kShiftLevelN = 2U;
constexpr std::uint8_t kShiftLevelR = 3U;
constexpr float kMotorSpeedFactorRpm = 0.1F;
constexpr float kMotorSpeedMaxRpm = 400.0F;
constexpr float kMotorSpeedMinRpm = -400.0F;

struct VehicleControlCommand
{
  std::uint8_t drive_mode{kDriveModeRemote};
  std::uint8_t shift_level{kShiftLevelN};
  float left_motor_speed_rpm{};
  float right_motor_speed_rpm{};
  bool brake_enable{true};
  std::uint8_t left_turn_light_req{};
  std::uint8_t right_turn_light_req{};
  std::uint8_t position_light_req{};
  std::uint8_t low_beam_req{};
  std::int8_t steering_angle_speed_valid{};
  std::uint8_t brake_force_command_valid{};
};

enum class VehicleControlEncodeResult
{
  OK,
  INVALID_OUTPUT,
  INVALID_DRIVE_MODE,
  INVALID_SHIFT_LEVEL,
  INVALID_MOTOR_SPEED,
  INVALID_TWO_BIT_FIELD,
  INVALID_STEERING_ANGLE_SPEED_VALID,
  INVALID_BRAKE_FORCE_COMMAND_VALID
};

VehicleControlCommand MakeSafeVehicleControlCommand();

VehicleControlEncodeResult EncodeVehicleControl(
  const VehicleControlCommand & command,
  CanFrameView * frame);

const char * VehicleControlEncodeResultToString(VehicleControlEncodeResult result);

}  // namespace autodrive_d02_can

#endif  // AUTODRIVE_D02_CAN__CAN_ENCODER_HPP_
