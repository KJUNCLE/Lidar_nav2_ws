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

#ifndef RTK_NODE__MESSAGE_CONVERSIONS_HPP_
#define RTK_NODE__MESSAGE_CONVERSIONS_HPP_

#include <cstdint>
#include <string>

#include "rtk_msgs/msg/rtk_data.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace rtk_node
{

sensor_msgs::msg::Imu to_imu_msg(
  const rtk_msgs::msg::RtkData & data,
  const std::string & frame_id);

sensor_msgs::msg::NavSatFix to_nav_sat_fix_msg(
  const rtk_msgs::msg::RtkData & data,
  const std::string & frame_id,
  std::uint16_t service_mask);

}  // namespace rtk_node

#endif  // RTK_NODE__MESSAGE_CONVERSIONS_HPP_
