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

#include <functional>
#include <memory>
#include <string>

#include "autodrive_d02_can/cmd_vel_converter.hpp"
#include "autodrive_d02_can_msgs/msg/vehicle_control_command.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autodrive_d02_can
{
namespace
{

autodrive_d02_can_msgs::msg::VehicleControlCommand ToMessage(
  const VehicleControlCommand & command,
  const builtin_interfaces::msg::Time & stamp)
{
  autodrive_d02_can_msgs::msg::VehicleControlCommand msg{};
  msg.header.stamp = stamp;
  msg.drive_mode = command.drive_mode;
  msg.shift_level = command.shift_level;
  msg.left_motor_speed_rpm = command.left_motor_speed_rpm;
  msg.right_motor_speed_rpm = command.right_motor_speed_rpm;
  msg.brake_enable = command.brake_enable;
  msg.left_turn_light_req = command.left_turn_light_req;
  msg.right_turn_light_req = command.right_turn_light_req;
  msg.position_light_req = command.position_light_req;
  msg.low_beam_req = command.low_beam_req;
  msg.steering_angle_speed_valid = command.steering_angle_speed_valid;
  msg.brake_force_command_valid = command.brake_force_command_valid;
  return msg;
}

}  // namespace

class D02CmdVelToVehicleControlNode final : public rclcpp::Node
{
public:
  explicit D02CmdVelToVehicleControlNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("d02_cmd_vel_to_vehicle_control", options)
  {
    config_.track_width_m = declare_parameter<double>("track_width_m", kDefaultTrackWidthM);
    config_.rpm_per_mps = declare_parameter<double>("rpm_per_mps", kDefaultRpmPerMps);
    config_.max_linear_mps = declare_parameter<double>("max_linear_mps", kDefaultMaxLinearMps);
    config_.max_angular_radps =
      declare_parameter<double>("max_angular_radps", kDefaultMaxAngularRadps);
    config_.stop_epsilon = declare_parameter<double>("stop_epsilon", kDefaultStopEpsilon);
    config_.shift_policy = declare_parameter<std::string>("shift_policy", "auto");

    command_pub_ =
      create_publisher<autodrive_d02_can_msgs::msg::VehicleControlCommand>(
      "vehicle_control_cmd", rclcpp::QoS(10));
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      std::bind(&D02CmdVelToVehicleControlNode::OnTwist, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "D02 cmd_vel converter: track_width_m=%.3f, rpm_per_mps=%.3f, "
      "max_linear_mps=%.3f, max_angular_radps=%.3f, shift_policy=%s",
      config_.track_width_m, config_.rpm_per_mps, config_.max_linear_mps,
      config_.max_angular_radps, config_.shift_policy.c_str());
  }

private:
  void OnTwist(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    CmdVelConversionResult result{CmdVelConversionResult::OK};
    const auto command = ConvertCmdVelToVehicleControl(
      msg->linear.x, msg->angular.z, config_, &result);

    if (result != CmdVelConversionResult::OK) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Invalid cmd_vel conversion (%s); publishing brake command",
        CmdVelConversionResultToString(result));
    }

    command_pub_->publish(ToMessage(command, now()));
  }

  CmdVelConversionConfig config_;
  rclcpp::Publisher<autodrive_d02_can_msgs::msg::VehicleControlCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
};

}  // namespace autodrive_d02_can

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autodrive_d02_can::D02CmdVelToVehicleControlNode>());
  rclcpp::shutdown();
  return 0;
}
