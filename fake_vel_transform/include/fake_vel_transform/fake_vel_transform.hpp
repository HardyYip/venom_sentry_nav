// Copyright 2025 Lihan Chen
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

#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace fake_vel_transform
{
class FakeVelTransform : public rclcpp::Node
{
public:
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  using Steady = std::chrono::steady_clock;
  using Tick = Steady::time_point;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, nav_msgs::msg::Path>;

  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const nav_msgs::msg::Path::ConstSharedPtr & local_plan);
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void cmdSpinCallback(example_interfaces::msg::Float32::SharedPtr msg);
  void stopCallback(example_interfaces::msg::Bool::SharedPtr msg);
  void timerCallback();
  void publishCommandLocked(bool synchronized);
  void publishStatusLocked(const std::string & state, const std::string & reason, Tick now);
  std::string faultReasonLocked(Tick now, double ros_now) const;
  bool controllerActiveLocked(Tick now) const;
  geometry_msgs::msg::Twist transformVelocity(
    const geometry_msgs::msg::Twist & twist, double yaw_diff) const;
  static double age(Tick now, Tick stamp);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_spin_sub_;
  rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr stop_sub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_filter_;
  message_filters::Subscriber<nav_msgs::msg::Path> local_plan_sub_filter_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string robot_base_frame_, fake_robot_base_frame_, odom_topic_, local_plan_topic_;
  std::string cmd_spin_topic_, input_cmd_vel_topic_, output_cmd_vel_topic_;
  std::string rotation_mode_;
  double spin_speed_{0.0};
  bool spin_on_zero_command_{true};
  double current_robot_base_angle_{0.0}, synchronized_angle_{0.0};
  bool safety_enabled_{true}, software_stop_{false}, controller_active_{false};
  bool pose_valid_{false}, plan_valid_{true}, spin_valid_{true}, command_valid_{true};
  bool faulted_{true}, recovery_pending_{false}, pair_valid_{false};
  double command_timeout_{0.5}, pose_timeout_{0.5}, sync_timeout_{0.5};
  double future_tolerance_{0.1}, max_sync_skew_{0.2}, diagnostics_period_{1.0};
  double odom_stamp_{0.0}, plan_stamp_{0.0}, pair_odom_stamp_{0.0}, pair_plan_stamp_{0.0};
  double pose_stamp_highwater_{0.0};
  builtin_interfaces::msg::Time odom_header_stamp_;
  Tick last_command_{}, last_odom_{}, last_pose_progress_{}, last_plan_{}, last_sync_{};
  Tick last_diagnostic_{};
  uint64_t command_sequence_{0}, recovery_sequence_{0};
  std::string last_status_;
  std::mutex mutex_;
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
};
}

#endif
