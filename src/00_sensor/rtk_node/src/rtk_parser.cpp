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

#include "rtk_node/rtk_parser.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace rtk_node
{
namespace
{

constexpr std::uint8_t kHeader0 = 0xbdU;
constexpr std::uint8_t kHeader1 = 0xdbU;
constexpr std::uint8_t kHeader2 = 0x0bU;
constexpr double kGpsTimeTickSeconds = 0.00025;

bool starts_with_header(const std::vector<std::uint8_t> & buffer)
{
  return buffer.size() >= 3U &&
         buffer[0] == kHeader0 &&
         buffer[1] == kHeader1 &&
         buffer[2] == kHeader2;
}

std::uint8_t xor_checksum(const std::vector<std::uint8_t> & buffer, std::size_t length)
{
  std::uint8_t checksum = 0U;
  for (std::size_t i = 0; i < length; ++i) {
    checksum ^= buffer[i];
  }
  return checksum;
}

bool checksum_valid(
  const std::vector<std::uint8_t> & buffer,
  std::size_t checksum_index)
{
  return buffer.size() > checksum_index &&
         xor_checksum(buffer, checksum_index) == buffer[checksum_index];
}

std::int16_t read_i16(const std::vector<std::uint8_t> & buffer, std::size_t offset)
{
  const std::uint16_t value =
    static_cast<std::uint16_t>(buffer[offset]) |
    static_cast<std::uint16_t>(buffer[offset + 1U] << 8U);
  if (value < 0x8000U) {
    return static_cast<std::int16_t>(value);
  }
  return static_cast<std::int16_t>(static_cast<std::int32_t>(value) - 0x10000);
}

std::int32_t read_i32(const std::vector<std::uint8_t> & buffer, std::size_t offset)
{
  const std::uint32_t value =
    static_cast<std::uint32_t>(buffer[offset]) |
    (static_cast<std::uint32_t>(buffer[offset + 1U]) << 8U) |
    (static_cast<std::uint32_t>(buffer[offset + 2U]) << 16U) |
    (static_cast<std::uint32_t>(buffer[offset + 3U]) << 24U);
  if (value < 0x80000000U) {
    return static_cast<std::int32_t>(value);
  }
  return static_cast<std::int32_t>(static_cast<std::int64_t>(value) - 0x100000000LL);
}

std::uint32_t read_u32(const std::vector<std::uint8_t> & buffer, std::size_t offset)
{
  return static_cast<std::uint32_t>(buffer[offset]) |
         (static_cast<std::uint32_t>(buffer[offset + 1U]) << 8U) |
         (static_cast<std::uint32_t>(buffer[offset + 2U]) << 16U) |
         (static_cast<std::uint32_t>(buffer[offset + 3U]) << 24U);
}

float attitude_degrees(std::int16_t value)
{
  return static_cast<float>(value) * 360.0F / 32768.0F;
}

float gyro_degrees_per_second(std::int16_t value)
{
  return static_cast<float>(value) * 300.0F / 32768.0F;
}

float accel_g(std::int16_t value)
{
  return static_cast<float>(value) * 12.0F / 32768.0F;
}

float velocity_meters_per_second(std::int16_t value)
{
  return static_cast<float>(value) * 100.0F / 32768.0F;
}

}  // namespace

void RtkParser::append(const std::uint8_t * data, std::size_t size)
{
  if (data == nullptr || size == 0U) {
    return;
  }
  buffer_.insert(buffer_.end(), data, data + size);
}

std::vector<rtk_msgs::msg::RtkData> RtkParser::parse_available(
  const builtin_interfaces::msg::Time & stamp)
{
  std::vector<rtk_msgs::msg::RtkData> messages;

  while (buffer_.size() >= 3U) {
    if (!starts_with_header(buffer_)) {
      buffer_.erase(buffer_.begin());
      ++dropped_bytes_;
      continue;
    }

    if (buffer_.size() < kPrimaryFrameLength) {
      break;
    }

    if (!checksum_valid(buffer_, kPrimaryFrameLength - 1U)) {
      buffer_.erase(buffer_.begin());
      ++dropped_bytes_;
      continue;
    }

    const bool full_checksum_ok =
      buffer_.size() >= kFullFrameLength &&
      checksum_valid(buffer_, kFullFrameLength - 1U);
    const std::size_t frame_length = full_checksum_ok ? kFullFrameLength : kPrimaryFrameLength;

    auto msg = rtk_msgs::msg::RtkData();
    msg.header.stamp = stamp;
    msg.primary_checksum_ok = true;
    msg.full_checksum_ok = full_checksum_ok;
    msg.raw_frame.assign(buffer_.begin(), buffer_.begin() + frame_length);

    msg.roll = attitude_degrees(read_i16(buffer_, 3U));
    msg.pitch = attitude_degrees(read_i16(buffer_, 5U));
    msg.azimuth = attitude_degrees(read_i16(buffer_, 7U));
    msg.x_angular_velocity = gyro_degrees_per_second(read_i16(buffer_, 9U));
    msg.y_angular_velocity = gyro_degrees_per_second(read_i16(buffer_, 11U));
    msg.z_angular_velocity = gyro_degrees_per_second(read_i16(buffer_, 13U));
    msg.x_acc = accel_g(read_i16(buffer_, 15U));
    msg.y_acc = accel_g(read_i16(buffer_, 17U));
    msg.z_acc = accel_g(read_i16(buffer_, 19U));

    msg.latitude = static_cast<double>(read_i32(buffer_, 21U)) * 1.0e-7;
    msg.longitude = static_cast<double>(read_i32(buffer_, 25U)) * 1.0e-7;
    msg.altitude = static_cast<double>(read_i32(buffer_, 29U)) * 1.0e-3;
    msg.north_velocity = velocity_meters_per_second(read_i16(buffer_, 33U));
    msg.east_velocity = velocity_meters_per_second(read_i16(buffer_, 35U));
    msg.down_velocity = velocity_meters_per_second(read_i16(buffer_, 37U));
    msg.ins_status = static_cast<std::uint8_t>(buffer_[39] & 0x0fU);

    msg.pdata1 = read_i16(buffer_, 46U);
    msg.pdata2 = read_i16(buffer_, 48U);
    msg.pdata3 = read_i16(buffer_, 50U);
    msg.sec_of_week = static_cast<double>(read_u32(buffer_, 52U)) * kGpsTimeTickSeconds;
    msg.pdata_type = buffer_[56];

    switch (msg.pdata_type) {
      case 0U:
        state_.latitude_std = std::exp(static_cast<float>(msg.pdata1) / 100.0F);
        state_.longitude_std = std::exp(static_cast<float>(msg.pdata2) / 100.0F);
        state_.altitude_std = std::exp(static_cast<float>(msg.pdata3) / 100.0F);
        break;
      case 1U:
        state_.north_velocity_std = std::exp(static_cast<float>(msg.pdata1) / 100.0F);
        state_.east_velocity_std = std::exp(static_cast<float>(msg.pdata2) / 100.0F);
        state_.down_velocity_std = std::exp(static_cast<float>(msg.pdata3) / 100.0F);
        break;
      case 2U:
        state_.roll_std = std::exp(static_cast<float>(msg.pdata1) / 100.0F);
        state_.pitch_std = std::exp(static_cast<float>(msg.pdata2) / 100.0F);
        state_.azimuth_std = std::exp(static_cast<float>(msg.pdata3) / 100.0F);
        break;
      case 22U:
        state_.temperature = static_cast<float>(msg.pdata1) * 200.0F / 32768.0F;
        break;
      case 32U:
        state_.position_type = static_cast<float>(msg.pdata1);
        state_.numsv = static_cast<float>(msg.pdata2);
        state_.heading_type = static_cast<float>(msg.pdata3);
        break;
      case 33U:
        state_.wheel_speed_status = static_cast<float>(msg.pdata2);
        break;
      default:
        break;
    }

    if (full_checksum_ok) {
      state_.gps_week_number = read_u32(buffer_, 58U);
    }

    msg.latitude_std = state_.latitude_std;
    msg.longitude_std = state_.longitude_std;
    msg.altitude_std = state_.altitude_std;
    msg.north_velocity_std = state_.north_velocity_std;
    msg.east_velocity_std = state_.east_velocity_std;
    msg.down_velocity_std = state_.down_velocity_std;
    msg.roll_std = state_.roll_std;
    msg.pitch_std = state_.pitch_std;
    msg.azimuth_std = state_.azimuth_std;
    msg.gps_week_number = state_.gps_week_number;
    msg.temperature = state_.temperature;
    msg.wheel_speed_status = state_.wheel_speed_status;
    msg.position_type = state_.position_type;
    msg.heading_type = state_.heading_type;
    msg.numsv = state_.numsv;

    messages.push_back(msg);
    ++parsed_frames_;
    buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_length));
  }

  return messages;
}

std::size_t RtkParser::parsed_frames() const
{
  return parsed_frames_;
}

std::size_t RtkParser::dropped_bytes() const
{
  return dropped_bytes_;
}

}  // namespace rtk_node
