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

#include "autodrive_d02_can/cmd_vel_converter.hpp"

#include <algorithm>
#include <cmath>

namespace autodrive_d02_can
{
namespace
{

bool IsFinitePositive(const double value)
{
  return std::isfinite(value) && value > 0.0;
}

bool IsFiniteNonNegative(const double value)
{
  return std::isfinite(value) && value >= 0.0;
}

double Clamp(const double value, const double lower, const double upper)
{
  return std::min(std::max(value, lower), upper);
}

bool IsStopped(
  const double linear_x_mps,
  const double angular_z_radps,
  const double stop_epsilon)
{
  return std::fabs(linear_x_mps) <= stop_epsilon &&
         std::fabs(angular_z_radps) <= stop_epsilon;
}

bool IsForwardMotion(const double left, const double right)
{
  return left >= 0.0 && right >= 0.0;
}

bool IsReverseMotion(const double left, const double right)
{
  return left <= 0.0 && right <= 0.0;
}

VehicleControlCommand MakeBrakeCommand()
{
  VehicleControlCommand command{};
  command.drive_mode = kDriveModeAuto;
  command.shift_level = kShiftLevelNone;
  command.left_motor_speed_rpm = 0.0F;
  command.right_motor_speed_rpm = 0.0F;
  command.brake_enable = true;
  return command;
}

}  // namespace

VehicleControlCommand ConvertCmdVelToVehicleControl(
  const double linear_x_mps,
  const double angular_z_radps,
  const CmdVelConversionConfig & config,
  CmdVelConversionResult * result)
{
  if (!IsFinitePositive(config.track_width_m) ||
    !IsFinitePositive(config.rpm_per_mps) ||
    !IsFiniteNonNegative(config.max_linear_mps) ||
    !IsFiniteNonNegative(config.max_angular_radps) ||
    !IsFinitePositive(config.stop_epsilon))
  {
    if (result != nullptr) {
      *result = CmdVelConversionResult::INVALID_CONFIG;
    }
    return MakeBrakeCommand();
  }

  if (config.shift_policy != "auto") {
    if (result != nullptr) {
      *result = CmdVelConversionResult::INVALID_SHIFT_POLICY;
    }
    return MakeBrakeCommand();
  }

  if (!std::isfinite(linear_x_mps) || !std::isfinite(angular_z_radps)) {
    if (result != nullptr) {
      *result = CmdVelConversionResult::INVALID_TWIST;
    }
    return MakeBrakeCommand();
  }

  const double linear = Clamp(
    linear_x_mps, -config.max_linear_mps, config.max_linear_mps);
  const double angular = Clamp(
    angular_z_radps, -config.max_angular_radps, config.max_angular_radps);

  if (IsStopped(linear, angular, config.stop_epsilon)) {
    if (result != nullptr) {
      *result = CmdVelConversionResult::OK;
    }
    return MakeBrakeCommand();
  }

  const double half_track = config.track_width_m * 0.5;
  const double left_velocity_mps = linear - angular * half_track;
  const double right_velocity_mps = linear + angular * half_track;

  VehicleControlCommand command{};
  command.drive_mode = kDriveModeAuto;
  command.brake_enable = false;

  if (IsForwardMotion(left_velocity_mps, right_velocity_mps)) {
    command.shift_level = kShiftLevelD;
    command.left_motor_speed_rpm =
      static_cast<float>(left_velocity_mps * config.rpm_per_mps);
    command.right_motor_speed_rpm =
      static_cast<float>(right_velocity_mps * config.rpm_per_mps);
  } else if (IsReverseMotion(left_velocity_mps, right_velocity_mps)) {
    command.shift_level = kShiftLevelR;
    command.left_motor_speed_rpm =
      static_cast<float>(std::fabs(left_velocity_mps) * config.rpm_per_mps);
    command.right_motor_speed_rpm =
      static_cast<float>(std::fabs(right_velocity_mps) * config.rpm_per_mps);
  } else {
    command.shift_level = kShiftLevelNone;
    command.left_motor_speed_rpm =
      static_cast<float>(left_velocity_mps * config.rpm_per_mps);
    command.right_motor_speed_rpm =
      static_cast<float>(right_velocity_mps * config.rpm_per_mps);
  }

  if (result != nullptr) {
    *result = CmdVelConversionResult::OK;
  }
  return command;
}

const char * CmdVelConversionResultToString(const CmdVelConversionResult result)
{
  switch (result) {
    case CmdVelConversionResult::OK:
      return "ok";
    case CmdVelConversionResult::INVALID_CONFIG:
      return "invalid config";
    case CmdVelConversionResult::INVALID_SHIFT_POLICY:
      return "invalid shift policy";
    case CmdVelConversionResult::INVALID_TWIST:
      return "invalid twist";
    default:
      return "unknown conversion result";
  }
}

}  // namespace autodrive_d02_can
