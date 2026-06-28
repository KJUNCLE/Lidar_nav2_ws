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
#include <cstdint>
#include <limits>

#include "autodrive_d02_can/can_encoder.hpp"
#include "autodrive_d02_can/cmd_vel_converter.hpp"

namespace autodrive_d02_can
{
namespace
{

CanFrameView Encode(const VehicleControlCommand & command)
{
  CanFrameView frame{};
  EXPECT_EQ(VehicleControlEncodeResult::OK, EncodeVehicleControl(command, &frame));
  return frame;
}

void ExpectConvertedFrame(
  const double linear_x_mps,
  const double angular_z_radps,
  const CmdVelConversionConfig & config,
  const std::array<std::uint8_t, 8> & expected_data)
{
  CmdVelConversionResult result{CmdVelConversionResult::INVALID_TWIST};
  const auto command = ConvertCmdVelToVehicleControl(
    linear_x_mps, angular_z_radps, config, &result);
  EXPECT_EQ(CmdVelConversionResult::OK, result);

  const auto frame = Encode(command);
  EXPECT_EQ(kVehicleControlCanId, frame.id);
  EXPECT_EQ(8U, frame.dlc);
  EXPECT_EQ(expected_data, frame.data);
}

TEST(D02CmdVelConverter, zeroVelocityEncodesBrakeFrame)
{
  CmdVelConversionConfig config{};

  ExpectConvertedFrame(
    0.0, 0.0, config,
    {0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x80U});
}

TEST(D02CmdVelConverter, forwardThreeKmhMatchesVerifiedFrame)
{
  CmdVelConversionConfig config{};
  config.max_linear_mps = 1.0;

  ExpectConvertedFrame(
    3.0 / 3.6, 0.0, config,
    {0x41U, 0x58U, 0x02U, 0x58U, 0x02U, 0x00U, 0x00U, 0x00U});
}

TEST(D02CmdVelConverter, reverseThreeKmhMatchesVerifiedFrame)
{
  CmdVelConversionConfig config{};
  config.max_linear_mps = 1.0;

  ExpectConvertedFrame(
    -3.0 / 3.6, 0.0, config,
    {0x43U, 0x58U, 0x02U, 0x58U, 0x02U, 0x00U, 0x00U, 0x00U});
}

TEST(D02CmdVelConverter, pureRotationGeneratesSignedOppositeWheelSpeeds)
{
  CmdVelConversionConfig config{};
  config.track_width_m = 0.39;
  config.max_angular_radps = 1.0;

  CmdVelConversionResult result{CmdVelConversionResult::INVALID_TWIST};
  const auto command = ConvertCmdVelToVehicleControl(0.0, 1.0, config, &result);

  EXPECT_EQ(CmdVelConversionResult::OK, result);
  EXPECT_EQ(kDriveModeAuto, command.drive_mode);
  EXPECT_EQ(kShiftLevelNone, command.shift_level);
  EXPECT_FALSE(command.brake_enable);
  EXPECT_LT(command.left_motor_speed_rpm, 0.0F);
  EXPECT_GT(command.right_motor_speed_rpm, 0.0F);
}

TEST(D02CmdVelConverter, oneStoppedWheelUsesDriveGearWhenMovingForward)
{
  CmdVelConversionConfig config{};
  config.track_width_m = 0.4;
  config.max_linear_mps = 1.0;
  config.max_angular_radps = 1.0;

  CmdVelConversionResult result{CmdVelConversionResult::INVALID_TWIST};
  const auto command = ConvertCmdVelToVehicleControl(0.2, 1.0, config, &result);

  EXPECT_EQ(CmdVelConversionResult::OK, result);
  EXPECT_EQ(kShiftLevelD, command.shift_level);
  EXPECT_FLOAT_EQ(0.0F, command.left_motor_speed_rpm);
  EXPECT_FLOAT_EQ(28.8F, command.right_motor_speed_rpm);
}

TEST(D02CmdVelConverter, clampsVelocityBeforeConverting)
{
  CmdVelConversionConfig config{};
  config.max_linear_mps = 0.26;
  config.max_angular_radps = 1.0;

  CmdVelConversionResult result{CmdVelConversionResult::INVALID_TWIST};
  const auto command = ConvertCmdVelToVehicleControl(10.0, 10.0, config, &result);

  EXPECT_EQ(CmdVelConversionResult::OK, result);
  EXPECT_EQ(kShiftLevelD, command.shift_level);
  EXPECT_FLOAT_EQ(static_cast<float>((0.26 - 0.39 * 0.5) * 72.0),
    command.left_motor_speed_rpm);
  EXPECT_FLOAT_EQ(static_cast<float>((0.26 + 0.39 * 0.5) * 72.0),
    command.right_motor_speed_rpm);
}

TEST(D02CmdVelConverter, invalidTwistReturnsBrakeCommand)
{
  CmdVelConversionConfig config{};

  CmdVelConversionResult result{CmdVelConversionResult::OK};
  const auto command = ConvertCmdVelToVehicleControl(
    std::numeric_limits<double>::quiet_NaN(), 0.0, config, &result);

  EXPECT_EQ(CmdVelConversionResult::INVALID_TWIST, result);
  const auto frame = Encode(command);
  EXPECT_EQ(
    (std::array<std::uint8_t, 8>{0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x80U}),
    frame.data);
}

TEST(D02CmdVelConverter, invalidConfigReturnsBrakeCommand)
{
  CmdVelConversionConfig config{};
  config.track_width_m = 0.0;

  CmdVelConversionResult result{CmdVelConversionResult::OK};
  const auto command = ConvertCmdVelToVehicleControl(0.1, 0.0, config, &result);

  EXPECT_EQ(CmdVelConversionResult::INVALID_CONFIG, result);
  const auto frame = Encode(command);
  EXPECT_EQ(
    (std::array<std::uint8_t, 8>{0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x80U}),
    frame.data);
}

TEST(D02CmdVelConverter, invalidShiftPolicyReturnsBrakeCommand)
{
  CmdVelConversionConfig config{};
  config.shift_policy = "manual";

  CmdVelConversionResult result{CmdVelConversionResult::OK};
  const auto command = ConvertCmdVelToVehicleControl(0.1, 0.0, config, &result);

  EXPECT_EQ(CmdVelConversionResult::INVALID_SHIFT_POLICY, result);
  const auto frame = Encode(command);
  EXPECT_EQ(
    (std::array<std::uint8_t, 8>{0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x80U}),
    frame.data);
}

}  // namespace
}  // namespace autodrive_d02_can
