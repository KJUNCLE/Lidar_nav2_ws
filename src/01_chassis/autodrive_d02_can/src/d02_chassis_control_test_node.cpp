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

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "can_msgs/msg/frame.hpp"
#include "rclcpp/rclcpp.hpp"

namespace autodrive_d02_can
{
namespace
{

using FrameData = std::array<std::uint8_t, 8>;

constexpr std::uint32_t kChassisControlCanId = 0x122U;
constexpr FrameData kBrakeFrame{{0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x80U}};
constexpr FrameData kForward3KmhFrame{{0x41U, 0x58U, 0x02U, 0x58U, 0x02U, 0x00U, 0x00U, 0x00U}};
constexpr FrameData kReverse3KmhFrame{{0x43U, 0x58U, 0x02U, 0x58U, 0x02U, 0x00U, 0x00U, 0x00U}};

struct TestStep
{
  std::string name;
  FrameData data;
  double duration_sec;
};

double PositiveOrDefault(const double value, const double fallback)
{
  return value > 0.0 ? value : fallback;
}

std::vector<TestStep> BuildSteps(
  const std::string & test_case,
  const double brake_duration_sec,
  const double motion_duration_sec)
{
  const TestStep brake{"brake", kBrakeFrame, brake_duration_sec};
  const TestStep forward{"forward_3kmh", kForward3KmhFrame, motion_duration_sec};
  const TestStep reverse{"reverse_3kmh", kReverse3KmhFrame, motion_duration_sec};

  if (test_case == "brake") {
    return {brake};
  }
  if (test_case == "forward_3kmh") {
    return {brake, forward, brake};
  }
  if (test_case == "reverse_3kmh") {
    return {brake, reverse, brake};
  }
  if (test_case == "sequence") {
    return {brake, forward, brake, reverse, brake};
  }

  throw std::invalid_argument(
          "test_case must be one of: brake, forward_3kmh, reverse_3kmh, sequence");
}

}  // namespace

class D02ChassisControlTestNode final : public rclcpp::Node
{
public:
  explicit D02ChassisControlTestNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("d02_chassis_control_test", options)
  {
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "can2");
    const auto test_case = declare_parameter<std::string>("test_case", "brake");
    send_period_sec_ = PositiveOrDefault(
      declare_parameter<double>("send_period_sec", 0.01), 0.01);
    const auto brake_duration_sec = PositiveOrDefault(
      declare_parameter<double>("brake_duration_sec", 1.0), 1.0);
    const auto motion_duration_sec = PositiveOrDefault(
      declare_parameter<double>("motion_duration_sec", 1.0), 1.0);

    steps_ = BuildSteps(test_case, brake_duration_sec, motion_duration_sec);
    frame_pub_ = create_publisher<can_msgs::msg::Frame>("to_can_bus", rclcpp::QoS(10));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(send_period_sec_));
    timer_ = create_wall_timer(period, [this]() {OnTimer();});

    RCLCPP_WARN(
      get_logger(),
      "Publishing raw 0x122 chassis test frames: test_case=%s, interface=%s",
      test_case.c_str(), output_frame_id_.c_str());
  }

private:
  void OnTimer()
  {
    if (!step_started_) {
      StartCurrentStep();
    }

    const auto elapsed_sec = (now() - step_start_time_).seconds();
    if (elapsed_sec >= steps_[current_step_index_].duration_sec) {
      ++current_step_index_;
      if (current_step_index_ >= steps_.size()) {
        PublishFrame(kBrakeFrame);
        RCLCPP_WARN(get_logger(), "Chassis control test complete; final brake frame sent");
        rclcpp::shutdown();
        return;
      }
      StartCurrentStep();
    }

    PublishFrame(steps_[current_step_index_].data);
  }

  void StartCurrentStep()
  {
    step_started_ = true;
    step_start_time_ = now();
    const auto & step = steps_[current_step_index_];
    RCLCPP_WARN(
      get_logger(), "Step %zu/%zu: %s for %.3f sec",
      current_step_index_ + 1U, steps_.size(), step.name.c_str(), step.duration_sec);
  }

  void PublishFrame(const FrameData & data)
  {
    can_msgs::msg::Frame msg{};
    msg.header.stamp = now();
    msg.header.frame_id = output_frame_id_;
    msg.id = kChassisControlCanId;
    msg.is_rtr = false;
    msg.is_extended = false;
    msg.is_error = false;
    msg.dlc = 8U;
    msg.data = data;
    frame_pub_->publish(msg);
  }

  std::string output_frame_id_;
  double send_period_sec_{0.01};
  bool step_started_{false};
  std::size_t current_step_index_{0U};
  rclcpp::Time step_start_time_{0, 0, RCL_ROS_TIME};
  std::vector<TestStep> steps_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr frame_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autodrive_d02_can

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<autodrive_d02_can::D02ChassisControlTestNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(rclcpp::get_logger("d02_chassis_control_test"), "%s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
