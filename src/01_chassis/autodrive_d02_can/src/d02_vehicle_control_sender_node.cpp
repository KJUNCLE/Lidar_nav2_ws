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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "autodrive_d02_can/can_encoder.hpp"
#include "autodrive_d02_can_msgs/msg/vehicle_control_command.hpp"
#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autodrive_d02_can
{
namespace
{

VehicleControlCommand ToCommand(
  const autodrive_d02_can_msgs::msg::VehicleControlCommand & msg)
{
  VehicleControlCommand command{};
  command.drive_mode = msg.drive_mode;
  command.shift_level = msg.shift_level;
  command.left_motor_speed_rpm = msg.left_motor_speed_rpm;
  command.right_motor_speed_rpm = msg.right_motor_speed_rpm;
  command.brake_enable = msg.brake_enable;
  command.left_turn_light_req = msg.left_turn_light_req;
  command.right_turn_light_req = msg.right_turn_light_req;
  command.position_light_req = msg.position_light_req;
  command.low_beam_req = msg.low_beam_req;
  command.steering_angle_speed_valid = msg.steering_angle_speed_valid;
  command.brake_force_command_valid = msg.brake_force_command_valid;
  return command;
}

can_msgs::msg::Frame ToFrameMessage(
  const CanFrameView & frame,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  can_msgs::msg::Frame msg{};
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.id = frame.id;
  msg.is_rtr = frame.is_rtr;
  msg.is_extended = frame.is_extended;
  msg.is_error = frame.is_error;
  msg.dlc = frame.dlc;
  msg.data = frame.data;
  return msg;
}

}  // namespace

class D02VehicleControlSenderNode final : public rclcpp::Node
{
public:
  explicit D02VehicleControlSenderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("d02_vehicle_control_sender", options),
    latest_command_(MakeSafeVehicleControlCommand())
  {
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "can2");
    send_period_sec_ = declare_parameter<double>("send_period_sec", 0.01);
    command_timeout_sec_ = declare_parameter<double>("command_timeout_sec", 0.1);

    if (send_period_sec_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "send_period_sec must be positive; using 0.01");
      send_period_sec_ = 0.01;
    }
    if (command_timeout_sec_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "command_timeout_sec must be positive; using 0.1");
      command_timeout_sec_ = 0.1;
    }

    frame_pub_ = create_publisher<can_msgs::msg::Frame>("to_can_bus", rclcpp::QoS(10));
    command_sub_ =
      create_subscription<autodrive_d02_can_msgs::msg::VehicleControlCommand>(
      "vehicle_control_cmd", rclcpp::QoS(10),
      std::bind(&D02VehicleControlSenderNode::OnCommand, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(send_period_sec_));
    timer_ = create_wall_timer(
      period, std::bind(&D02VehicleControlSenderNode::OnTimer, this));
  }

private:
  void OnCommand(const autodrive_d02_can_msgs::msg::VehicleControlCommand::SharedPtr msg)
  {
    const auto command = ToCommand(*msg);
    CanFrameView frame{};
    const auto result = EncodeVehicleControl(command, &frame);
    if (result != VehicleControlEncodeResult::OK) {
      RCLCPP_WARN(
        get_logger(), "Invalid vehicle control command (%s); switching to safe command",
        VehicleControlEncodeResultToString(result));
      latest_command_ = MakeSafeVehicleControlCommand();
    } else {
      latest_command_ = command;
      has_valid_command_ = true;
    }

    last_command_time_ = now();
  }

  void OnTimer()
  {
    const auto current_time = now();
    VehicleControlCommand command = latest_command_;

    if (!has_valid_command_ ||
      (current_time - last_command_time_).seconds() > command_timeout_sec_)
    {
      command = MakeSafeVehicleControlCommand();
    }

    CanFrameView frame{};
    const auto result = EncodeVehicleControl(command, &frame);
    if (result != VehicleControlEncodeResult::OK) {
      RCLCPP_ERROR(
        get_logger(), "Failed to encode vehicle control frame: %s",
        VehicleControlEncodeResultToString(result));
      return;
    }

    frame_pub_->publish(ToFrameMessage(frame, current_time, output_frame_id_));
  }

  std::string output_frame_id_;
  double send_period_sec_{0.01};
  double command_timeout_sec_{0.1};
  bool has_valid_command_{false};
  VehicleControlCommand latest_command_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr frame_pub_;
  rclcpp::Subscription<autodrive_d02_can_msgs::msg::VehicleControlCommand>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autodrive_d02_can

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autodrive_d02_can::D02VehicleControlSenderNode>());
  rclcpp::shutdown();
  return 0;
}
