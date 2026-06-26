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

#include "autodrive_d02_can/can_decoder.hpp"
#include "autodrive_d02_can_msgs/msg/battery_feedback.hpp"
#include "autodrive_d02_can_msgs/msg/target_speed_feedback.hpp"
#include "autodrive_d02_can_msgs/msg/vehicle_status_feedback.hpp"
#include "autodrive_d02_can_msgs/msg/warning_level_feedback.hpp"
#include "autodrive_d02_can_msgs/msg/wheel_speed_feedback.hpp"
#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autodrive_d02_can
{
namespace
{

CanFrameView ToFrameView(const can_msgs::msg::Frame & msg)
{
  CanFrameView frame{};
  frame.id = msg.id;
  frame.is_extended = msg.is_extended;
  frame.is_rtr = msg.is_rtr;
  frame.is_error = msg.is_error;
  frame.dlc = msg.dlc;
  frame.data = msg.data;
  return frame;
}

autodrive_d02_can_msgs::msg::TargetSpeedFeedback ToMessage(
  const TargetSpeedFeedback & feedback,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  autodrive_d02_can_msgs::msg::TargetSpeedFeedback msg{};
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.can_id = feedback.can_id;
  msg.hardware_target_speed_raw = feedback.hardware_target_speed_raw;
  msg.hardware_target_speed_kmh = feedback.hardware_target_speed_kmh;
  msg.scu_target_speed_feedback_raw = feedback.scu_target_speed_feedback_raw;
  msg.scu_target_speed_feedback_kmh = feedback.scu_target_speed_feedback_kmh;
  msg.vehicle_target_speed_raw = feedback.vehicle_target_speed_raw;
  msg.vehicle_target_speed_kmh = feedback.vehicle_target_speed_kmh;
  msg.vehicle_target_speed_rpm_raw = feedback.vehicle_target_speed_rpm_raw;
  msg.vehicle_target_speed_rpm = feedback.vehicle_target_speed_rpm;
  return msg;
}

autodrive_d02_can_msgs::msg::WheelSpeedFeedback ToMessage(
  const WheelSpeedFeedback & feedback,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  autodrive_d02_can_msgs::msg::WheelSpeedFeedback msg{};
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.can_id = feedback.can_id;
  msg.front_left_rpm_raw = feedback.front_left_rpm_raw;
  msg.front_left_rpm = feedback.front_left_rpm;
  msg.front_right_rpm_raw = feedback.front_right_rpm_raw;
  msg.front_right_rpm = feedback.front_right_rpm;
  msg.rear_left_rpm_raw = feedback.rear_left_rpm_raw;
  msg.rear_left_rpm = feedback.rear_left_rpm;
  msg.rear_right_rpm_raw = feedback.rear_right_rpm_raw;
  msg.rear_right_rpm = feedback.rear_right_rpm;
  return msg;
}

autodrive_d02_can_msgs::msg::BatteryFeedback ToMessage(
  const BatteryFeedback & feedback,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  autodrive_d02_can_msgs::msg::BatteryFeedback msg{};
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.can_id = feedback.can_id;
  msg.bms_voltage_raw = feedback.bms_voltage_raw;
  msg.bms_voltage_v = feedback.bms_voltage_v;
  msg.bms_current_raw = feedback.bms_current_raw;
  msg.bms_current_a = feedback.bms_current_a;
  msg.bms_soc_raw = feedback.bms_soc_raw;
  msg.bms_soc_percent = feedback.bms_soc_percent;
  return msg;
}

autodrive_d02_can_msgs::msg::VehicleStatusFeedback ToMessage(
  const VehicleStatusFeedback & feedback,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  autodrive_d02_can_msgs::msg::VehicleStatusFeedback msg{};
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.can_id = feedback.can_id;
  msg.ccu_shift_level_status = feedback.ccu_shift_level_status;
  msg.ccu_parking_status = feedback.ccu_parking_status;
  msg.ccu_ignition_status = feedback.ccu_ignition_status;
  msg.ccu_drive_mode_shift_button = feedback.ccu_drive_mode_shift_button;
  msg.steering_wheel_direction = feedback.steering_wheel_direction;
  msg.ccu_steering_wheel_angle_raw = feedback.ccu_steering_wheel_angle_raw;
  msg.ccu_steering_wheel_angle_deg = feedback.ccu_steering_wheel_angle_deg;
  msg.ccu_vehicle_speed_raw = feedback.ccu_vehicle_speed_raw;
  msg.ccu_vehicle_speed_kmh = feedback.ccu_vehicle_speed_kmh;
  msg.ccu_drive_mode = feedback.ccu_drive_mode;
  msg.remote_brake_request_status = feedback.remote_brake_request_status;
  msg.emergency_brake_request_status = feedback.emergency_brake_request_status;
  msg.scu_brake_signal_status = feedback.scu_brake_signal_status;
  msg.touch_brake_request_status = feedback.touch_brake_request_status;
  msg.handle_brake_request_status = feedback.handle_brake_request_status;
  msg.handle_mode_flag_status = feedback.handle_mode_flag_status;
  msg.left_turn_light_status = feedback.left_turn_light_status;
  msg.right_turn_light_status = feedback.right_turn_light_status;
  msg.position_light_status = feedback.position_light_status;
  msg.low_beam_status = feedback.low_beam_status;
  return msg;
}

autodrive_d02_can_msgs::msg::WarningLevelFeedback ToMessage(
  const WarningLevelFeedback & feedback,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  autodrive_d02_can_msgs::msg::WarningLevelFeedback msg{};
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.can_id = feedback.can_id;
  msg.bms_soc_warning = feedback.bms_soc_warning;
  msg.mcu_disconnect_warning = feedback.mcu_disconnect_warning;
  msg.mcu_motor_warning = feedback.mcu_motor_warning;
  msg.mcu_overspeed_warning = feedback.mcu_overspeed_warning;
  msg.turn_disconnect_warning = feedback.turn_disconnect_warning;
  msg.turn_lock_warning = feedback.turn_lock_warning;
  msg.turn_unstoppable_warning = feedback.turn_unstoppable_warning;
  msg.sas_angle_error = feedback.sas_angle_error;
  msg.brake_error_warning = feedback.brake_error_warning;
  return msg;
}

}  // namespace

class D02VehicleFeedbackDecoderNode final : public rclcpp::Node
{
public:
  explicit D02VehicleFeedbackDecoderNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("d02_vehicle_feedback_decoder", options)
  {
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "can2");

    target_speed_pub_ =
      create_publisher<autodrive_d02_can_msgs::msg::TargetSpeedFeedback>(
      "target_speed_feedback", rclcpp::QoS(10));
    wheel_speed_pub_ =
      create_publisher<autodrive_d02_can_msgs::msg::WheelSpeedFeedback>(
      "wheel_speed_feedback", rclcpp::QoS(10));
    battery_pub_ =
      create_publisher<autodrive_d02_can_msgs::msg::BatteryFeedback>(
      "battery_feedback", rclcpp::QoS(10));
    vehicle_status_pub_ =
      create_publisher<autodrive_d02_can_msgs::msg::VehicleStatusFeedback>(
      "vehicle_status_feedback", rclcpp::QoS(10));
    warning_level_pub_ =
      create_publisher<autodrive_d02_can_msgs::msg::WarningLevelFeedback>(
      "warning_level_feedback", rclcpp::QoS(10));

    frame_sub_ = create_subscription<can_msgs::msg::Frame>(
      "from_can_bus", rclcpp::QoS(500),
      std::bind(&D02VehicleFeedbackDecoderNode::OnFrame, this, std::placeholders::_1));
  }

private:
  void OnFrame(const can_msgs::msg::Frame::SharedPtr msg)
  {
    const auto frame = ToFrameView(*msg);
    const auto stamp = msg->header.stamp;

    TargetSpeedFeedback target_speed{};
    if (DecodeTargetSpeedFeedback(frame, &target_speed)) {
      target_speed_pub_->publish(ToMessage(target_speed, stamp, output_frame_id_));
      return;
    }

    WheelSpeedFeedback wheel_speed{};
    if (DecodeWheelSpeedFeedback(frame, &wheel_speed)) {
      wheel_speed_pub_->publish(ToMessage(wheel_speed, stamp, output_frame_id_));
      return;
    }

    BatteryFeedback battery{};
    if (DecodeBatteryFeedback(frame, &battery)) {
      battery_pub_->publish(ToMessage(battery, stamp, output_frame_id_));
      return;
    }

    VehicleStatusFeedback vehicle_status{};
    if (DecodeVehicleStatusFeedback(frame, &vehicle_status)) {
      vehicle_status_pub_->publish(ToMessage(vehicle_status, stamp, output_frame_id_));
      return;
    }

    WarningLevelFeedback warning_level{};
    if (DecodeWarningLevelFeedback(frame, &warning_level)) {
      warning_level_pub_->publish(ToMessage(warning_level, stamp, output_frame_id_));
    }
  }

  std::string output_frame_id_;
  rclcpp::Publisher<autodrive_d02_can_msgs::msg::TargetSpeedFeedback>::SharedPtr target_speed_pub_;
  rclcpp::Publisher<autodrive_d02_can_msgs::msg::WheelSpeedFeedback>::SharedPtr wheel_speed_pub_;
  rclcpp::Publisher<autodrive_d02_can_msgs::msg::BatteryFeedback>::SharedPtr battery_pub_;
  rclcpp::Publisher<autodrive_d02_can_msgs::msg::VehicleStatusFeedback>::SharedPtr
    vehicle_status_pub_;
  rclcpp::Publisher<autodrive_d02_can_msgs::msg::WarningLevelFeedback>::SharedPtr
    warning_level_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr frame_sub_;
};

}  // namespace autodrive_d02_can

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<autodrive_d02_can::D02VehicleFeedbackDecoderNode>());
  rclcpp::shutdown();
  return 0;
}
