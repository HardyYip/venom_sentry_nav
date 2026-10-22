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

#include "fake_vel_transform/fake_vel_transform.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{
namespace
{
double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec + stamp.nanosec * 1e-9;
}
bool finiteTwist(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.angular.z);
}
bool zeroTwist(const geometry_msgs::msg::Twist & twist)
{
  return std::abs(twist.linear.x) < 1e-5 && std::abs(twist.linear.y) < 1e-5 &&
         std::abs(twist.angular.z) < 1e-5;
}
}

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  robot_base_frame_ = declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  fake_robot_base_frame_ = declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "odom");
  local_plan_topic_ = declare_parameter<std::string>("local_plan_topic", "local_plan");
  cmd_spin_topic_ = declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  input_cmd_vel_topic_ = declare_parameter<std::string>("input_cmd_vel_topic", "");
  output_cmd_vel_topic_ = declare_parameter<std::string>("output_cmd_vel_topic", "");
  spin_speed_ = declare_parameter<double>("init_spin_speed", 0.0);
  spin_on_zero_command_ = declare_parameter<bool>("spin_on_zero_command", true);
  rotation_mode_ = declare_parameter<std::string>("rotation_mode", "additive");
  if (rotation_mode_ != "additive" && rotation_mode_ != "heading") {
    throw std::invalid_argument("rotation_mode must be additive or heading");
  }
  safety_enabled_ = declare_parameter<bool>("safety_enabled", true);
  command_timeout_ = declare_parameter<double>("command_timeout", 0.5);
  pose_timeout_ = declare_parameter<double>("pose_timeout", 0.5);
  sync_timeout_ = declare_parameter<double>("sync_timeout", 0.5);
  future_tolerance_ = declare_parameter<double>("future_stamp_tolerance", 0.1);
  max_sync_skew_ = declare_parameter<double>("max_sync_stamp_skew", 0.2);
  diagnostics_period_ = declare_parameter<double>("diagnostics_period", 1.0);
  const auto stop_topic = declare_parameter<std::string>("safety_stop_topic", "safety_stop");
  const auto status_topic = declare_parameter<std::string>("safety_status_topic", "safety_status");
  for (const auto value : {command_timeout_, pose_timeout_, sync_timeout_, max_sync_skew_, diagnostics_period_}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("Velocity safety timeouts must be positive and finite");
    }
  }
  if (!std::isfinite(future_tolerance_) || future_tolerance_ < 0.0 || !std::isfinite(spin_speed_)) {
    throw std::invalid_argument("Invalid velocity safety parameter");
  }
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  cmd_vel_chassis_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);
  status_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(status_topic, 10);
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10, std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  cmd_spin_sub_ = create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  stop_sub_ = create_subscription<example_interfaces::msg::Bool>(
    stop_topic, 1, std::bind(&FakeVelTransform::stopCallback, this, std::placeholders::_1));
  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&FakeVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));
  timer_ = create_wall_timer(std::chrono::milliseconds(20), std::bind(&FakeVelTransform::timerCallback, this));
}

double FakeVelTransform::age(Tick now, Tick stamp)
{
  if (stamp == Tick{}) {return std::numeric_limits<double>::infinity();}
  return std::chrono::duration<double>(now - stamp).count();
}

bool FakeVelTransform::controllerActiveLocked(Tick now) const
{
  return controller_active_ && (safety_enabled_ || age(now, last_plan_) < 0.5);
}

std::string FakeVelTransform::faultReasonLocked(Tick now, double ros_now) const
{
  if (software_stop_) {return "software_stop";}
  if (!safety_enabled_) {return "";}
  if (!spin_valid_) {return "invalid_spin";}
  if (!pose_valid_) {return "pose_missing_or_invalid";}
  if (age(now, last_odom_) > pose_timeout_) {return "odometry_receive_timeout";}
  if (age(now, last_pose_progress_) > pose_timeout_) {return "pose_stamp_stalled";}
  if (ros_now - odom_stamp_ > pose_timeout_) {return "pose_source_stale";}
  if (odom_stamp_ - ros_now > future_tolerance_) {return "pose_source_future";}
  if (!command_valid_) {return "invalid_command";}
  if (!latest_cmd_vel_) {return "command_missing";}
  if (age(now, last_command_) > command_timeout_) {return "command_receive_timeout";}
  if (!plan_valid_) {return "empty_or_invalid_plan";}
  if (controllerActiveLocked(now)) {
    if (age(now, last_plan_) > sync_timeout_) {return "plan_receive_timeout";}
    if (ros_now - plan_stamp_ > pose_timeout_) {return "plan_source_stale";}
    if (plan_stamp_ - ros_now > future_tolerance_) {return "plan_source_future";}
    if (!pair_valid_ || age(now, last_sync_) > sync_timeout_) {return "synchronization_timeout";}
    if (pair_odom_stamp_ - ros_now > future_tolerance_ ||
      pair_plan_stamp_ - ros_now > future_tolerance_) {return "synchronized_pose_future";}
    if (ros_now - pair_odom_stamp_ > pose_timeout_ || ros_now - pair_plan_stamp_ > pose_timeout_) {
      return "synchronized_pose_stale";
    }
  }
  return "";
}

void FakeVelTransform::publishStatusLocked(
  const std::string & state, const std::string & reason, Tick now)
{
  const std::string status_key = state + ":" + reason;
  if (status_key == last_status_ && age(now, last_diagnostic_) < diagnostics_period_) {return;}
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = get_fully_qualified_name() + std::string("/velocity_safety");
  status.hardware_id = "navigation_velocity_chain";
  status.level = state == "ACTIVE" ? status.OK : (state == "RECOVERING" ? status.WARN : status.ERROR);
  status.message = status_key;
  const auto add = [&status](const std::string & key, const std::string & value) {
      diagnostic_msgs::msg::KeyValue entry;
      entry.key = key;
      entry.value = value;
      status.values.push_back(entry);
    };
  add("state", state);
  add("reason", reason);
  add("safety_enabled", safety_enabled_ ? "true" : "false");
  add("command_receive_age_s", std::to_string(age(now, last_command_)));
  add("odometry_receive_age_s", std::to_string(age(now, last_odom_)));
  add("pose_progress_age_s", std::to_string(age(now, last_pose_progress_)));
  add("pose_source_age_s", std::to_string(get_clock()->now().seconds() - odom_stamp_));
  add("synchronization_age_s", std::to_string(age(now, last_sync_)));
  add("requested_spin", std::to_string(spin_speed_));
  add("rotation_mode", rotation_mode_);
  add("controller_active", controllerActiveLocked(now) ? "true" : "false");
  array.status.push_back(status);
  status_pub_->publish(array);
  if (status_key != last_status_) {
    RCLCPP_INFO(get_logger(), "Velocity safety: %s", status_key.c_str());
  }
  last_status_ = status_key;
  last_diagnostic_ = now;
}

void FakeVelTransform::publishCommandLocked(bool synchronized)
{
  const auto now = Steady::now();
  const auto reason = faultReasonLocked(now, get_clock()->now().seconds());
  if (!reason.empty()) {
    if (!faulted_) {
      latest_cmd_vel_.reset();
      pair_valid_ = false;
    }
    faulted_ = true;
    recovery_pending_ = false;
    cmd_vel_chassis_pub_->publish(geometry_msgs::msg::Twist{});
    publishStatusLocked("STOPPED", reason, now);
    return;
  }
  if (faulted_) {
    if (!recovery_pending_) {
      recovery_pending_ = true;
      recovery_sequence_ = command_sequence_;
    }
    if (command_sequence_ <= recovery_sequence_) {
      cmd_vel_chassis_pub_->publish(geometry_msgs::msg::Twist{});
      publishStatusLocked("RECOVERING", "waiting_for_new_command", now);
      return;
    }
    faulted_ = false;
    recovery_pending_ = false;
  }
  if (!controllerActiveLocked(now) || synchronized) {
    const double angle = synchronized ? synchronized_angle_ : current_robot_base_angle_;
    if (latest_cmd_vel_) {cmd_vel_chassis_pub_->publish(transformVelocity(*latest_cmd_vel_, angle));}
  }
  publishStatusLocked("ACTIVE", "healthy", now);
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  spin_valid_ = std::isfinite(msg->data);
  if (spin_valid_) {spin_speed_ = msg->data;}
  publishCommandLocked(false);
}

void FakeVelTransform::stopCallback(const example_interfaces::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  software_stop_ = msg->data;
  publishCommandLocked(false);
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  pose_valid_ = std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
    std::isfinite(norm) && norm > 1e-12;
  if (pose_valid_) {
    const double stamp = stampSeconds(msg->header.stamp);
    if (stamp <= get_clock()->now().seconds() + future_tolerance_ &&
      (stamp > pose_stamp_highwater_ || last_odom_ == Tick{})) {
      last_pose_progress_ = Steady::now();
      pose_stamp_highwater_ = stamp;
    }
    odom_stamp_ = stamp;
    odom_header_stamp_ = msg->header.stamp;
    last_odom_ = Steady::now();
    current_robot_base_angle_ = tf2::getYaw(q);
  }
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  controller_active_ = true;
  plan_valid_ = !msg->poses.empty();
  plan_stamp_ = stampSeconds(msg->header.stamp);
  last_plan_ = Steady::now();
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ++command_sequence_;
  command_valid_ = finiteTwist(*msg);
  last_command_ = Steady::now();
  latest_cmd_vel_ = msg;
  if (zeroTwist(*msg)) {controller_active_ = false;}
  publishCommandLocked(false);
}

void FakeVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
  const nav_msgs::msg::Path::ConstSharedPtr & plan)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pair_odom_stamp_ = stampSeconds(odom->header.stamp);
  pair_plan_stamp_ = stampSeconds(plan->header.stamp);
  const double angle = tf2::getYaw(odom->pose.pose.orientation);
  pair_valid_ = std::isfinite(angle) && !plan->poses.empty() &&
    std::abs(pair_odom_stamp_ - pair_plan_stamp_) <= max_sync_skew_;
  if (pair_valid_) {
    last_sync_ = Steady::now();
    synchronized_angle_ = angle;
  }
  publishCommandLocked(true);
}

void FakeVelTransform::timerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  publishCommandLocked(false);
  const auto now = Steady::now();
  if (!pose_valid_ || (safety_enabled_ &&
    (age(now, last_odom_) > pose_timeout_ || age(now, last_pose_progress_) > pose_timeout_ ||
    get_clock()->now().seconds() - odom_stamp_ > pose_timeout_ ||
    odom_stamp_ - get_clock()->now().seconds() > future_tolerance_))) {return;}
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = odom_header_stamp_;
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist & twist, double yaw_diff) const
{
  geometry_msgs::msg::Twist result;
  const bool add_spin = rotation_mode_ == "additive" && (spin_on_zero_command_ || !zeroTwist(twist));
  result.angular.z = twist.angular.z + (add_spin ? spin_speed_ : 0.0);
  result.linear.x = twist.linear.x * std::cos(yaw_diff) + twist.linear.y * std::sin(yaw_diff);
  result.linear.y = -twist.linear.x * std::sin(yaw_diff) + twist.linear.y * std::cos(yaw_diff);
  return result;
}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
