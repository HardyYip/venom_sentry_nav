// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>

#include <dwb_core/trajectory_critic.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rmul_c072_planner
{
class TerminalGoalCritic : public dwb_core::TrajectoryCritic
{
public:
  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D &) override
  {
    goal_ = goal;
    const auto distance = std::hypot(pose.x - goal.x, pose.y - goal.y);
    const auto weight = std::clamp(1.0 - distance / activation_distance_m, 0.0, 1.0);
    // Score in cell units to match DWB's existing endpoint scores, but retain
    // sub-cell distance and use the final global goal supplied by DWB.
    weight_per_metre_ = weight / costmap_ros_->getCostmap()->getResolution();
    return true;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (weight_per_metre_ == 0.0 || trajectory.poses.empty()) {
      return 0.0;
    }
    const auto & end = trajectory.poses.back();
    return weight_per_metre_ * std::hypot(end.x - goal_.x, end.y - goal_.y);
  }

private:
  static constexpr double activation_distance_m = 0.4;
  geometry_msgs::msg::Pose2D goal_;
  double weight_per_metre_{0.0};
};
}  // namespace rmul_c072_planner

PLUGINLIB_EXPORT_CLASS(rmul_c072_planner::TerminalGoalCritic, dwb_core::TrajectoryCritic)
