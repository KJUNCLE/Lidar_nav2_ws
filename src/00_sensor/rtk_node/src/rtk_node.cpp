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

#include "rtk_node/rtk_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtk_node/message_conversions.hpp"

namespace rtk_node
{
namespace
{

int positive_or_default(int value, int default_value)
{
  return value > 0 ? value : default_value;
}

std::uint16_t clamp_to_u16(std::int64_t value)
{
  return static_cast<std::uint16_t>(
    std::clamp<std::int64_t>(value, 0, 65535));
}

builtin_interfaces::msg::Time to_builtin_time(const rclcpp::Time & time)
{
  const auto total_nanoseconds = time.nanoseconds();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(total_nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(total_nanoseconds % 1000000000LL);
  return stamp;
}

}  // namespace

RtkNode::RtkNode(const rclcpp::NodeOptions & options)
: Node("rtk_node", options),
  port_(declare_parameter<std::string>("port", "/dev/ttyUSB0")),
  baud_rate_(positive_or_default(declare_parameter<int>("baud_rate", 230400), 230400)),
  read_chunk_size_(static_cast<std::size_t>(
      positive_or_default(declare_parameter<int>("read_chunk_size", 512), 512))),
  reconnect_interval_(std::chrono::milliseconds(
      positive_or_default(declare_parameter<int>("reconnect_interval_ms", 5000), 5000))),
  last_reconnect_attempt_(std::chrono::steady_clock::time_point::min()),
  imu_frame_id_(declare_parameter<std::string>("imu_frame_id", "imu_link")),
  gps_frame_id_(declare_parameter<std::string>("gps_frame_id", "gps_link")),
  gnss_service_mask_(clamp_to_u16(declare_parameter<int>("gnss_service_mask", 15)))
{
  const auto imu_topic = declare_parameter<std::string>("imu_topic", "/rtk/imu");
  const auto fix_topic = declare_parameter<std::string>("fix_topic", "/rtk/fix");
  const auto data_topic = declare_parameter<std::string>("data_topic", "/rtk/data");
  const auto poll_period_ms =
    positive_or_default(declare_parameter<int>("poll_period_ms", 1), 1);

  const auto qos = rclcpp::SensorDataQoS();
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic, qos);
  fix_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic, qos);
  data_publisher_ = create_publisher<rtk_msgs::msg::RtkData>(data_topic, qos);

  open_serial();
  timer_ = create_wall_timer(
    std::chrono::milliseconds(poll_period_ms),
    std::bind(&RtkNode::timer_callback, this));
}

bool RtkNode::open_serial()
{
  last_reconnect_attempt_ = std::chrono::steady_clock::now();
  std::string error;
  if (serial_.open(port_, baud_rate_, &error)) {
    RCLCPP_INFO(get_logger(), "Opened RTK serial port %s at %d baud", port_.c_str(), baud_rate_);
    return true;
  }

  RCLCPP_WARN(get_logger(), "Unable to open RTK serial port %s: %s", port_.c_str(), error.c_str());
  return false;
}

void RtkNode::timer_callback()
{
  if (!serial_.is_open()) {
    const auto now = std::chrono::steady_clock::now();
    if (now - last_reconnect_attempt_ >= reconnect_interval_) {
      open_serial();
    }
    return;
  }

  std::vector<std::uint8_t> buffer(read_chunk_size_);
  std::string error;
  const ssize_t bytes_read = serial_.read(buffer.data(), buffer.size(), &error);
  if (bytes_read < 0) {
    RCLCPP_WARN(get_logger(), "Error reading RTK serial port %s: %s", port_.c_str(), error.c_str());
    serial_.close();
    return;
  }
  if (bytes_read == 0) {
    return;
  }

  parser_.append(buffer.data(), static_cast<std::size_t>(bytes_read));
  auto messages = parser_.parse_available(to_builtin_time(now()));
  for (auto & data : messages) {
    data.header.frame_id = gps_frame_id_;
    imu_publisher_->publish(to_imu_msg(data, imu_frame_id_));
    fix_publisher_->publish(to_nav_sat_fix_msg(data, gps_frame_id_, gnss_service_mask_));
    data_publisher_->publish(data);
  }
}

}  // namespace rtk_node

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rtk_node::RtkNode>());
  rclcpp::shutdown();
  return 0;
}
