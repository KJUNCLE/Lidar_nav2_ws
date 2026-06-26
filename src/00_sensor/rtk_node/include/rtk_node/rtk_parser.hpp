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

#ifndef RTK_NODE__RTK_PARSER_HPP_
#define RTK_NODE__RTK_PARSER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rtk_msgs/msg/rtk_data.hpp"

namespace rtk_node
{

class RtkParser
{
public:
  static constexpr std::size_t kPrimaryFrameLength = 58U;
  static constexpr std::size_t kFullFrameLength = 63U;

  void append(const std::uint8_t * data, std::size_t size);
  std::vector<rtk_msgs::msg::RtkData> parse_available(
    const builtin_interfaces::msg::Time & stamp);

  std::size_t parsed_frames() const;
  std::size_t dropped_bytes() const;

private:
  struct State
  {
    float latitude_std = 0.0F;
    float longitude_std = 0.0F;
    float altitude_std = 0.0F;
    float north_velocity_std = 0.0F;
    float east_velocity_std = 0.0F;
    float down_velocity_std = 0.0F;
    float roll_std = 0.0F;
    float pitch_std = 0.0F;
    float azimuth_std = 0.0F;
    std::uint32_t gps_week_number = 0U;
    float temperature = 0.0F;
    float wheel_speed_status = 0.0F;
    float ins_status = 0.0F;
    float position_type = 0.0F;
    float heading_type = 0.0F;
    float numsv = 0.0F;
  };

  std::vector<std::uint8_t> buffer_;
  State state_;
  std::size_t parsed_frames_ = 0U;
  std::size_t dropped_bytes_ = 0U;
};

}  // namespace rtk_node

#endif  // RTK_NODE__RTK_PARSER_HPP_
