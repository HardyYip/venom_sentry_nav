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
  void onInit() override
  {
    const auto node = node_.lock();
    if (node) {
      node->get_parameter("general_goal_checker.xy_goal_tolerance", goal_tolerance_);
    }
  }

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D & goal, const nav_2d_msgs::msg::Path2D &) override
  {
    goal_ = goal;
    const auto distance = distanceToGoal(pose);
    finishing_ = distance > goal_tolerance_ && distance <= 2.0 * goal_tolerance_;
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
    const auto endpoint_score = weight_per_metre_ * distanceToGoal(trajectory.poses.back());
    if (!finishing_) {
      return endpoint_score;
    }
    // Just outside tolerance, a coarse velocity sample can pass through the
    // goal circle even when its rollout endpoint overshoots it. Give those
    // low-speed trajectories precedence while retaining every obstacle critic.
    const auto speed = std::hypot(trajectory.velocity.x, trajectory.velocity.y);
    const bool settles_into_goal =
      speed <= finishing_speed_mps &&
      std::any_of(trajectory.poses.begin(), trajectory.poses.end(),
        [this](const auto & pose) {return distanceToGoal(pose) <= goal_tolerance_ * 0.95;});
    return endpoint_score + (settles_into_goal ? 0.0 : missed_goal_penalty_cells);
  }

private:
  double distanceToGoal(const geometry_msgs::msg::Pose2D & pose) const
  {
    return std::hypot(pose.x - goal_.x, pose.y - goal_.y);
  }

  static constexpr double activation_distance_m = 0.4;
  static constexpr double finishing_speed_mps = 0.20;
  static constexpr double missed_goal_penalty_cells = 3.0;
  geometry_msgs::msg::Pose2D goal_;
  double goal_tolerance_{0.08};
  double weight_per_metre_{0.0};
  bool finishing_{false};
};
}  // namespace rmul_c072_planner

PLUGINLIB_EXPORT_CLASS(rmul_c072_planner::TerminalGoalCritic, dwb_core::TrajectoryCritic)
