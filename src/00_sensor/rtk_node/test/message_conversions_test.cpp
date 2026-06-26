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

#include "gtest/gtest.h"
#include "rtk_node/message_conversions.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace
{

constexpr double kTolerance = 1.0e-6;

rtk_msgs::msg::RtkData make_data()
{
  rtk_msgs::msg::RtkData data;
  data.header.stamp.sec = 12;
  data.header.stamp.nanosec = 345;
  data.header.frame_id = "rtk_raw";
  data.latitude = 31.2345678;
  data.longitude = 121.2345678;
  data.altitude = 12.3;
  data.position_type = 4.0F;
  return data;
}

}  // namespace

TEST(MessageConversions, ImuUsesStandardUnitsAndQuaternion)
{
  auto data = make_data();
  data.azimuth = 90.0F;
  data.x_angular_velocity = 180.0F;
  data.y_angular_velocity = -90.0F;
  data.z_angular_velocity = 45.0F;
  data.x_acc = 1.0F;
  data.y_acc = -0.5F;
  data.z_acc = 0.25F;
  data.roll_std = 1.0F;
  data.pitch_std = 2.0F;
  data.azimuth_std = 3.0F;

  const auto msg = rtk_node::to_imu_msg(data, "imu_link");

  EXPECT_EQ(msg.header.stamp.sec, 12);
  EXPECT_EQ(msg.header.frame_id, "imu_link");
  EXPECT_NEAR(msg.orientation.z, std::sqrt(0.5), kTolerance);
  EXPECT_NEAR(msg.orientation.w, std::sqrt(0.5), kTolerance);
  EXPECT_NEAR(msg.angular_velocity.x, M_PI, kTolerance);
  EXPECT_NEAR(msg.angular_velocity.y, -M_PI / 2.0, kTolerance);
  EXPECT_NEAR(msg.angular_velocity.z, M_PI / 4.0, kTolerance);
  EXPECT_NEAR(msg.linear_acceleration.x, 9.80665, kTolerance);
  EXPECT_NEAR(msg.linear_acceleration.y, -4.903325, kTolerance);
  EXPECT_NEAR(msg.linear_acceleration.z, 2.4516625, kTolerance);
  EXPECT_NEAR(msg.orientation_covariance[0], std::pow(M_PI / 180.0, 2.0), kTolerance);
  EXPECT_NEAR(msg.orientation_covariance[4], std::pow(2.0 * M_PI / 180.0, 2.0), kTolerance);
  EXPECT_NEAR(msg.orientation_covariance[8], std::pow(3.0 * M_PI / 180.0, 2.0), kTolerance);
}

TEST(MessageConversions, NavSatFixUsesEnuCovarianceAndFixStatus)
{
  auto data = make_data();
  data.latitude_std = 2.0F;
  data.longitude_std = 3.0F;
  data.altitude_std = 4.0F;

  const auto msg = rtk_node::to_nav_sat_fix_msg(data, "gps_link", 15U);

  EXPECT_EQ(msg.header.frame_id, "gps_link");
  EXPECT_EQ(msg.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
  EXPECT_EQ(msg.status.service, 15U);
  EXPECT_DOUBLE_EQ(msg.latitude, data.latitude);
  EXPECT_DOUBLE_EQ(msg.longitude, data.longitude);
  EXPECT_DOUBLE_EQ(msg.altitude, data.altitude);
  EXPECT_NEAR(msg.position_covariance[0], 9.0, kTolerance);
  EXPECT_NEAR(msg.position_covariance[4], 4.0, kTolerance);
  EXPECT_NEAR(msg.position_covariance[8], 16.0, kTolerance);
  EXPECT_EQ(
    msg.position_covariance_type,
    sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN);
}

TEST(MessageConversions, NavSatFixReportsNoFixWhenPositionTypeInvalid)
{
  auto data = make_data();
  data.position_type = 0.0F;

  const auto msg = rtk_node::to_nav_sat_fix_msg(data, "gps_link", 1U);

  EXPECT_EQ(msg.status.status, sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX);
  EXPECT_EQ(msg.position_covariance_type, sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN);
}
