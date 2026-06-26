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

#include "rtk_node/message_conversions.hpp"

#include <array>
#include <cmath>

#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace rtk_node
{
namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kStandardGravity = 9.80665;

double deg_to_rad(double value)
{
  return value * kDegreesToRadians;
}

bool valid_std(float value)
{
  return std::isfinite(value) && value > 0.0F;
}

bool valid_geodetic_position(const rtk_msgs::msg::RtkData & data)
{
  return data.position_type > 0.0F &&
         std::isfinite(data.latitude) &&
         std::isfinite(data.longitude) &&
         data.latitude >= -90.0 &&
         data.latitude <= 90.0 &&
         data.longitude >= -180.0 &&
         data.longitude <= 180.0;
}

}  // namespace

sensor_msgs::msg::Imu to_imu_msg(
  const rtk_msgs::msg::RtkData & data,
  const std::string & frame_id)
{
  sensor_msgs::msg::Imu msg;
  msg.header = data.header;
  msg.header.frame_id = frame_id;

  const double roll = deg_to_rad(data.roll);
  const double pitch = deg_to_rad(data.pitch);
  const double yaw = deg_to_rad(data.azimuth);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  msg.orientation.w = cr * cp * cy + sr * sp * sy;
  msg.orientation.x = sr * cp * cy - cr * sp * sy;
  msg.orientation.y = cr * sp * cy + sr * cp * sy;
  msg.orientation.z = cr * cp * sy - sr * sp * cy;

  msg.angular_velocity.x = deg_to_rad(data.x_angular_velocity);
  msg.angular_velocity.y = deg_to_rad(data.y_angular_velocity);
  msg.angular_velocity.z = deg_to_rad(data.z_angular_velocity);

  msg.linear_acceleration.x = data.x_acc * kStandardGravity;
  msg.linear_acceleration.y = data.y_acc * kStandardGravity;
  msg.linear_acceleration.z = data.z_acc * kStandardGravity;

  if (valid_std(data.roll_std)) {
    msg.orientation_covariance[0] = std::pow(deg_to_rad(data.roll_std), 2.0);
  }
  if (valid_std(data.pitch_std)) {
    msg.orientation_covariance[4] = std::pow(deg_to_rad(data.pitch_std), 2.0);
  }
  if (valid_std(data.azimuth_std)) {
    msg.orientation_covariance[8] = std::pow(deg_to_rad(data.azimuth_std), 2.0);
  }

  return msg;
}

sensor_msgs::msg::NavSatFix to_nav_sat_fix_msg(
  const rtk_msgs::msg::RtkData & data,
  const std::string & frame_id,
  std::uint16_t service_mask)
{
  sensor_msgs::msg::NavSatFix msg;
  msg.header = data.header;
  msg.header.frame_id = frame_id;
  msg.status.service = service_mask;
  msg.status.status = valid_geodetic_position(data) ?
    sensor_msgs::msg::NavSatStatus::STATUS_FIX :
    sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  msg.latitude = data.latitude;
  msg.longitude = data.longitude;
  msg.altitude = data.altitude;

  if (valid_std(data.latitude_std) &&
    valid_std(data.longitude_std) &&
    valid_std(data.altitude_std))
  {
    msg.position_covariance[0] = std::pow(data.longitude_std, 2.0);
    msg.position_covariance[4] = std::pow(data.latitude_std, 2.0);
    msg.position_covariance[8] = std::pow(data.altitude_std, 2.0);
    msg.position_covariance_type =
      sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  } else {
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }

  return msg;
}

}  // namespace rtk_node
