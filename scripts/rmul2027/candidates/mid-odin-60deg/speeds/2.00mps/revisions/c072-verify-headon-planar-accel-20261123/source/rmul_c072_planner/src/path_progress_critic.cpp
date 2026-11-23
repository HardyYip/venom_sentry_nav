// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <limits>

#include <dwb_core/trajectory_critic.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "inflation_escape.hpp"

namespace rmul_c072_planner
{
class PathProgressCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    const auto node = node_.lock();
    clock_ = node->get_clock();
  }

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D & path) override
  {
    constexpr double activation_speed = 0.25;
    constexpr double lookahead = 0.35;
    weight_ = 0.;
    if (path.poses.size() < 2 || std::hypot(pose.x - goal.x, pose.y - goal.y) < 0.45) {
      return true;
    }
    const auto speed = std::hypot(velocity.x, velocity.y);
    if (!needsProgressAssist(speed, activation_speed)) {
      return true;
    }

    std::size_t closest = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.poses.size(); ++i) {
      const auto distance = std::hypot(path.poses[i].x - pose.x, path.poses[i].y - pose.y);
      if (distance < closest_distance) {
        closest = i;
        closest_distance = distance;
      }
    }
    waypoint_ = path.poses.back();
    double remaining = lookahead;
    for (std::size_t i = closest + 1; i < path.poses.size(); ++i) {
      const auto & from = path.poses[i - 1];
      const auto & to = path.poses[i];
      const auto dx = to.x - from.x;
      const auto dy = to.y - from.y;
      const auto length = std::hypot(dx, dy);
      if (length >= remaining) {
        waypoint_.x = from.x + dx * remaining / length;
        waypoint_.y = from.y + dy * remaining / length;
        break;
      }
      remaining -= length;
    }
    const auto * map = costmap_ros_->getCostmap();
    weight_ = (1.0 - speed / activation_speed) / map->getResolution();
    // Renew the pull of the nearby route if the robot has stopped just
    // outside the inflation reserve. The collision critics still reject
    // unsafe trajectories and the output controller still enforces braking.
    constexpr double edge_range = 0.65;
    const double clearance = inflation_escape::clearance(*map, {pose.x, pose.y}, edge_range);
    if (clearance >= inflation_escape::normal_clearance_m && clearance < edge_range) {
      weight_ *= 4.0;
      if (const auto node = node_.lock()) {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *clock_, 3000,
          "C072 restoring route progress at %.3f m local clearance", clearance);
      }
    }
    return true;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (weight_ == 0.0 || trajectory.poses.empty()) {
      return 0.0;
    }
    const auto & end = trajectory.poses.back();
    return weight_ * std::hypot(end.x - waypoint_.x, end.y - waypoint_.y);
  }

private:
  bool needsProgressAssist(double speed, double activation_speed)
  {
    constexpr double stationary_speed = 0.05;
    constexpr double stationary_time = 0.65;
    constexpr double assist_time = 0.90;
    const double now = clock_->now().seconds();
    if (now < last_time_) {
      stationary_since_ = -1.0;
      assist_until_ = -1.0;
    }
    last_time_ = now;
    if (speed >= activation_speed) {
      stationary_since_ = -1.0;
      assist_until_ = -1.0;
      return false;
    }
    if (now < assist_until_) {
      return true;
    }
    if (speed > stationary_speed) {
      stationary_since_ = -1.0;
      return false;
    }
    if (stationary_since_ < 0.0) {
      stationary_since_ = now;
      return false;
    }
    if (now - stationary_since_ < stationary_time) {
      return false;
    }
    // Give ordinary acceleration and replanning a chance between short
    // assists. A clear straight path never needs the stationary score.
    stationary_since_ = -1.0;
    assist_until_ = now + assist_time;
    return true;
  }

  rclcpp::Clock::SharedPtr clock_;
  double last_time_{-1.0};
  double stationary_since_{-1.0};
  double assist_until_{-1.0};
  geometry_msgs::msg::Pose2D waypoint_;
  double weight_{0.0};
};
}  // namespace rmul_c072_planner

PLUGINLIB_EXPORT_CLASS(rmul_c072_planner::PathProgressCritic, dwb_core::TrajectoryCritic)
