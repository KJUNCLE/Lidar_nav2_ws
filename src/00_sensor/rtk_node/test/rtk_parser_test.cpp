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

#include <cmath>
#include <cstdint>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "gtest/gtest.h"
#include "rtk_node/rtk_parser.hpp"

namespace
{

constexpr double kTolerance = 1.0e-5;

void write_i16(std::vector<std::uint8_t> & frame, std::size_t offset, std::int16_t value)
{
  frame[offset] = static_cast<std::uint8_t>(value & 0xff);
  frame[offset + 1U] = static_cast<std::uint8_t>((value >> 8) & 0xff);
}

void write_i32(std::vector<std::uint8_t> & frame, std::size_t offset, std::int32_t value)
{
  frame[offset] = static_cast<std::uint8_t>(value & 0xff);
  frame[offset + 1U] = static_cast<std::uint8_t>((value >> 8) & 0xff);
  frame[offset + 2U] = static_cast<std::uint8_t>((value >> 16) & 0xff);
  frame[offset + 3U] = static_cast<std::uint8_t>((value >> 24) & 0xff);
}

void write_u32(std::vector<std::uint8_t> & frame, std::size_t offset, std::uint32_t value)
{
  frame[offset] = static_cast<std::uint8_t>(value & 0xff);
  frame[offset + 1U] = static_cast<std::uint8_t>((value >> 8) & 0xff);
  frame[offset + 2U] = static_cast<std::uint8_t>((value >> 16) & 0xff);
  frame[offset + 3U] = static_cast<std::uint8_t>((value >> 24) & 0xff);
}

std::uint8_t xor_checksum(const std::vector<std::uint8_t> & frame, std::size_t length)
{
  std::uint8_t checksum = 0U;
  for (std::size_t i = 0; i < length; ++i) {
    checksum ^= frame[i];
  }
  return checksum;
}

std::vector<std::uint8_t> make_frame(
  std::uint8_t pdata_type = 0U,
  std::int16_t pdata1 = 100,
  std::int16_t pdata2 = 200,
  std::int16_t pdata3 = 300,
  bool full_frame = false)
{
  std::vector<std::uint8_t> frame(
    full_frame ? rtk_node::RtkParser::kFullFrameLength : rtk_node::RtkParser::kPrimaryFrameLength,
    0U);
  frame[0] = 0xbdU;
  frame[1] = 0xdbU;
  frame[2] = 0x0bU;

  write_i16(frame, 3, 8192);
  write_i16(frame, 5, -4096);
  write_i16(frame, 7, 16384);
  write_i16(frame, 9, 3277);
  write_i16(frame, 11, -1638);
  write_i16(frame, 13, 819);
  write_i16(frame, 15, 2731);
  write_i16(frame, 17, -1365);
  write_i16(frame, 19, 683);
  write_i32(frame, 21, 312345678);
  write_i32(frame, 25, 1212345678);
  write_i32(frame, 29, 123456);
  write_i16(frame, 33, 3277);
  write_i16(frame, 35, -1638);
  write_i16(frame, 37, 819);
  frame[39] = 0x17U;
  write_i16(frame, 46, pdata1);
  write_i16(frame, 48, pdata2);
  write_i16(frame, 50, pdata3);
  write_u32(frame, 52, 4000U);
  frame[56] = pdata_type;
  frame[57] = xor_checksum(frame, 57U);

  if (full_frame) {
    write_u32(frame, 58, 2240U);
    frame[62] = xor_checksum(frame, 62U);
  }

  return frame;
}

builtin_interfaces::msg::Time stamp()
{
  builtin_interfaces::msg::Time value;
  value.sec = 42;
  value.nanosec = 123456789U;
  return value;
}

}  // namespace

TEST(RtkParser, ValidPrimaryFrameParsesCoreFields)
{
  rtk_node::RtkParser parser;
  const auto frame = make_frame();

  parser.append(frame.data(), frame.size());
  const auto messages = parser.parse_available(stamp());

  ASSERT_EQ(messages.size(), 1U);
  const auto & msg = messages.front();
  EXPECT_TRUE(msg.primary_checksum_ok);
  EXPECT_FALSE(msg.full_checksum_ok);
  EXPECT_EQ(msg.raw_frame.size(), rtk_node::RtkParser::kPrimaryFrameLength);
  EXPECT_EQ(msg.header.stamp.sec, 42);
  EXPECT_NEAR(msg.latitude, 31.2345678, kTolerance);
  EXPECT_NEAR(msg.longitude, 121.2345678, kTolerance);
  EXPECT_NEAR(msg.altitude, 123.456, kTolerance);
  EXPECT_NEAR(msg.roll, 90.0, kTolerance);
  EXPECT_NEAR(msg.pitch, -45.0, kTolerance);
  EXPECT_NEAR(msg.azimuth, 180.0, kTolerance);
  EXPECT_NEAR(msg.x_angular_velocity, 3277.0 * 300.0 / 32768.0, kTolerance);
  EXPECT_NEAR(msg.x_acc, 2731.0 * 12.0 / 32768.0, kTolerance);
  EXPECT_NEAR(msg.north_velocity, 3277.0 * 100.0 / 32768.0, kTolerance);
  EXPECT_EQ(msg.ins_status, 7U);
  EXPECT_EQ(msg.pdata_type, 0U);
  EXPECT_EQ(msg.pdata1, 100);
  EXPECT_NEAR(msg.latitude_std, std::exp(1.0), kTolerance);
  EXPECT_NEAR(msg.longitude_std, std::exp(2.0), kTolerance);
  EXPECT_NEAR(msg.altitude_std, std::exp(3.0), kTolerance);
  EXPECT_NEAR(msg.sec_of_week, 1.0, kTolerance);
  EXPECT_EQ(parser.parsed_frames(), 1U);
}

TEST(RtkParser, FullFrameParsesGpsWeekAndKeepsRawBytes)
{
  rtk_node::RtkParser parser;
  const auto frame = make_frame(0U, 100, 200, 300, true);

  parser.append(frame.data(), frame.size());
  const auto messages = parser.parse_available(stamp());

  ASSERT_EQ(messages.size(), 1U);
  EXPECT_TRUE(messages.front().full_checksum_ok);
  EXPECT_EQ(messages.front().gps_week_number, 2240U);
  EXPECT_EQ(messages.front().raw_frame.size(), rtk_node::RtkParser::kFullFrameLength);
}

TEST(RtkParser, FragmentedInputResynchronizesAfterNoise)
{
  rtk_node::RtkParser parser;
  const auto frame = make_frame();
  const std::vector<std::uint8_t> noise{0x12U, 0x34U};

  parser.append(noise.data(), noise.size());
  parser.append(frame.data(), 10U);
  EXPECT_TRUE(parser.parse_available(stamp()).empty());

  parser.append(frame.data() + 10U, frame.size() - 10U);
  const auto messages = parser.parse_available(stamp());

  ASSERT_EQ(messages.size(), 1U);
  EXPECT_EQ(parser.dropped_bytes(), 2U);
}

TEST(RtkParser, BadChecksumFrameIsDroppedAndFollowingFrameRecovers)
{
  rtk_node::RtkParser parser;
  auto bad_frame = make_frame();
  bad_frame[57] ^= 0xffU;
  const auto good_frame = make_frame();

  parser.append(bad_frame.data(), bad_frame.size());
  parser.append(good_frame.data(), good_frame.size());
  const auto messages = parser.parse_available(stamp());

  ASSERT_EQ(messages.size(), 1U);
  EXPECT_TRUE(messages.front().primary_checksum_ok);
}

TEST(RtkParser, PDataStatePersistsAcrossFrames)
{
  rtk_node::RtkParser parser;
  const auto status_frame = make_frame(32U, 4, 18, 5);
  const auto wheel_frame = make_frame(33U, 0, 7, 0);

  parser.append(status_frame.data(), status_frame.size());
  parser.append(wheel_frame.data(), wheel_frame.size());
  const auto messages = parser.parse_available(stamp());

  ASSERT_EQ(messages.size(), 2U);
  EXPECT_NEAR(messages.back().position_type, 4.0, kTolerance);
  EXPECT_NEAR(messages.back().numsv, 18.0, kTolerance);
  EXPECT_NEAR(messages.back().heading_type, 5.0, kTolerance);
  EXPECT_NEAR(messages.back().wheel_speed_status, 7.0, kTolerance);
}
