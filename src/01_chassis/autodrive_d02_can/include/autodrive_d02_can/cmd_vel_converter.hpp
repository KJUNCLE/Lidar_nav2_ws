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

#ifndef AUTODRIVE_D02_CAN__CMD_VEL_CONVERTER_HPP_
#define AUTODRIVE_D02_CAN__CMD_VEL_CONVERTER_HPP_

#include <string>

#include "autodrive_d02_can/can_encoder.hpp"

namespace autodrive_d02_can
{

constexpr double kDefaultTrackWidthM = 0.39;
constexpr double kDefaultRpmPerMps = 72.0;
constexpr double kDefaultMaxLinearMps = 0.26;
constexpr double kDefaultMaxAngularRadps = 1.0;
constexpr double kDefaultStopEpsilon = 1.0e-4;

struct CmdVelConversionConfig
{
  double track_width_m{kDefaultTrackWidthM};
  double rpm_per_mps{kDefaultRpmPerMps};
  double max_linear_mps{kDefaultMaxLinearMps};
  double max_angular_radps{kDefaultMaxAngularRadps};
  double stop_epsilon{kDefaultStopEpsilon};
  std::string shift_policy{"auto"};
};

enum class CmdVelConversionResult
{
  OK,
  INVALID_CONFIG,
  INVALID_SHIFT_POLICY,
  INVALID_TWIST
};

VehicleControlCommand ConvertCmdVelToVehicleControl(
  double linear_x_mps,
  double angular_z_radps,
  const CmdVelConversionConfig & config,
  CmdVelConversionResult * result);

const char * CmdVelConversionResultToString(CmdVelConversionResult result);

}  // namespace autodrive_d02_can

#endif  // AUTODRIVE_D02_CAN__CMD_VEL_CONVERTER_HPP_
