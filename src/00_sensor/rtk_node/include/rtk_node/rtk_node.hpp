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

#ifndef RTK_NODE__RTK_NODE_HPP_
#define RTK_NODE__RTK_NODE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rtk_msgs/msg/rtk_data.hpp"
#include "rtk_node/rtk_parser.hpp"
#include "rtk_node/serial_port.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace rtk_node
{

class RtkNode : public rclcpp::Node
{
public:
  explicit RtkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void timer_callback();
  bool open_serial();

  std::string port_;
  int baud_rate_;
  std::size_t read_chunk_size_;
  std::chrono::milliseconds reconnect_interval_;
  std::chrono::steady_clock::time_point last_reconnect_attempt_;

  std::string imu_frame_id_;
  std::string gps_frame_id_;
  std::uint16_t gnss_service_mask_;

  SerialPort serial_;
  RtkParser parser_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr fix_publisher_;
  rclcpp::Publisher<rtk_msgs::msg::RtkData>::SharedPtr data_publisher_;
};

}  // namespace rtk_node

#endif  // RTK_NODE__RTK_NODE_HPP_
