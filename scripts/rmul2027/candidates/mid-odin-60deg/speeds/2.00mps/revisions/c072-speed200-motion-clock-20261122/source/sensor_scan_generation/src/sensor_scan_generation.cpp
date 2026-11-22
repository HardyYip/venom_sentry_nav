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

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensor_scan_generation
{

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);

  require_valid_tf_ = this->declare_parameter<bool>("require_valid_tf", false);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_laser_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_scan", 2);
  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);

  rmw_qos_profile_t qos_profile = {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false};

  odometry_sub_.subscribe(this, "lidar_odometry", qos_profile);
  laser_cloud_sub_.subscribe(this, "registered_scan", qos_profile);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odometry_sub_, laser_cloud_sub_);
  sync_->registerCallback(
    std::bind(
      &SensorScanGenerationNode::laserCloudAndOdometryHandler, this, std::placeholders::_1,
      std::placeholders::_2));
}

void SensorScanGenerationNode::laserCloudAndOdometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & pcd_msg)
{
  tf2::Transform tf_lidar_to_chassis;
  tf2::Transform tf_odom_to_chassis;
  tf2::Transform tf_odom_to_robot_base;
  tf2::Transform tf_odom_to_lidar;

  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);
  if (!getTransform(
      lidar_frame_, robot_base_frame_, pcd_msg->header.stamp, tf_lidar_to_robot_base_) ||
    !getTransform(lidar_frame_, base_frame_, pcd_msg->header.stamp, tf_lidar_to_chassis))
  {
    return;
  }

  tf_odom_to_chassis = tf_odom_to_lidar * tf_lidar_to_chassis;
  tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base_;

  publishTransform(
    tf_odom_to_chassis, odometry_msg->header.frame_id, base_frame_, pcd_msg->header.stamp);
  publishOdometry(
    tf_odom_to_robot_base, odometry_msg->header.frame_id, robot_base_frame_,
    pcd_msg->header.stamp, odometry_msg->header.stamp);

  sensor_msgs::msg::PointCloud2 out;
  geometry_msgs::msg::TransformStamped scan_to_lidar;
  scan_to_lidar.header = pcd_msg->header;
  scan_to_lidar.header.frame_id = lidar_frame_;
  scan_to_lidar.transform = tf2::toMsg(tf_odom_to_lidar.inverse());
  tf2::doTransform(*pcd_msg, out, scan_to_lidar);
  pub_laser_cloud_->publish(out);
}

bool SensorScanGenerationNode::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const rclcpp::Time & time, tf2::Transform & transform)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(0.5));
    tf2::fromMsg(transform_stamped.transform, transform);
    return true;
  } catch (tf2::TransformException & ex) {
    if (require_valid_tf_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "TF lookup failed: %s. Dropping scan/odometry pair.", ex.what());
      return false;
    }
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s. Returning identity.", ex.what());
    transform = tf2::Transform::getIdentity();
    return true;
  }
}

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp, const rclcpp::Time & pose_stamp)
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());

  // The poses are sampled by LiDAR odometry. Callback arrival times include
  // queuing and processing jitter and cannot be used to measure their velocity.
  if (previous_odometry_stamp_) {
    const double dt = (pose_stamp - *previous_odometry_stamp_).seconds();
    if (dt > 0.0001 && dt <= 0.30) {
      const auto linear_velocity =
        (origin - previous_odometry_transform_.getOrigin()) / dt;
      auto rotation_difference =
        transform.getRotation() * previous_odometry_transform_.getRotation().inverse();
      if (rotation_difference.w() < 0.0) {
        rotation_difference.setValue(
          -rotation_difference.x(), -rotation_difference.y(),
          -rotation_difference.z(), -rotation_difference.w());
      }
      const auto angular_velocity =
        rotation_difference.getAxis() * rotation_difference.getAngle() / dt;
      out.twist.twist.linear.x = linear_velocity.x();
      out.twist.twist.linear.y = linear_velocity.y();
      out.twist.twist.linear.z = linear_velocity.z();
      out.twist.twist.angular.x = angular_velocity.x();
      out.twist.twist.angular.y = angular_velocity.y();
      out.twist.twist.angular.z = angular_velocity.z();
    }
    if (dt == 0.0) {
      return;
    }
  }

  previous_odometry_transform_ = transform;
  previous_odometry_stamp_ = pose_stamp;
  pub_chassis_odometry_->publish(out);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
