// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>

#include <dwb_core/exceptions.hpp>
#include <dwb_core/trajectory_critic.hpp>
#include <nav2_core/controller.hpp>
#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "inflation_escape.hpp"

namespace rmul_c072_planner
{
namespace escape = inflation_escape;

class InflationAwareController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    base_ = controllers_.createSharedInstance(
      "pb_omni_pid_pursuit_controller::CollisionAwareOmniController");
    base_->configure(parent, name, tf, costmap);
    auto node = parent.lock();
    clock_ = node->get_clock();
    logger_ = node->get_logger();
    escape_plan_ = node->create_publisher<nav_msgs::msg::Path>("local_plan", rclcpp::QoS(1));
    costmap_ = std::move(costmap);
    tf_ = std::move(tf);
    nominal_speed_ = node->get_parameter(name + ".max_speed_xy").as_double();
    speed_limit_ = nominal_speed_;
    horizon_ = node->get_parameter(name + ".sim_time").as_double();
    braking_rate_ = std::min(-node->get_parameter(name + ".decel_lim_x").as_double(),
      -node->get_parameter(name + ".decel_lim_y").as_double());
    if (!(std::isfinite(braking_rate_) && braking_rate_ > 0.0)) {
      throw nav2_core::PlannerException("Inflation approach needs a positive braking rate");
    }
    prediction_ = critics_.createSharedInstance("rmul_c072_planner::PredictedObstacleCritic");
    prediction_->initialize(node, "EscapePredictedObstacle", name, costmap_);
    for (const auto & p : costmap_->getRobotFootprint()) {
      radius_ = std::max(radius_, std::hypot(p.x, p.y));
    }
    if (costmap_->getRobotFootprint().empty() || radius_ >= escape::normal_clearance_m) {
      throw nav2_core::PlannerException("Inflation recovery needs the configured physical footprint");
    }
  }

  void activate() override
  {
    base_->activate();
    escape_plan_->on_activate();
  }
  void deactivate() override
  {
    resetEscape();
    escape_plan_->on_deactivate();
    base_->deactivate();
  }
  void cleanup() override
  {
    prediction_.reset();
    escape_plan_.reset();
    base_->cleanup();
    base_.reset();
    plan_.poses.clear();
    costmap_.reset();
    tf_.reset();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    plan_ = path;
    base_->setPlan(path);
  }

  void setSpeedLimit(const double & speed, const bool & percentage) override
  {
    base_->setSpeedLimit(speed, percentage);
    speed_limit_ = speed == nav2_costmap_2d::NO_SPEED_LIMIT ? nominal_speed_ :
      std::clamp(percentage ? nominal_speed_ * speed / 100.0 : speed, 0.0, nominal_speed_);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    const escape::Point current{pose.pose.position.x, pose.pose.position.y};
    if (pose.header.frame_id != costmap_->getGlobalFrameID()) {
      throw nav2_core::PlannerException("Inflation recovery requires the controller costmap frame");
    }
    const auto stamp = rclcpp::Time(pose.header.stamp).seconds();
    updateSpeed(current, stamp);
    auto * grid = costmap_->getCostmap();
    bool needs_escape;
    {
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
      const auto threshold = escaping_ ? escape::escape_end_clearance_m : escape::normal_clearance_m;
      needs_escape = escape::clearance(*grid, current, threshold) < threshold ||
        escape::inscribed(*grid, current);
    }

    if (needs_escape) {
      auto output = escapeCommand(pose, current, stamp);
      if (published_escape_plan_ && !escaping_) {
        nav_msgs::msg::Path stopped;
        stopped.header = pose.header;
        escape_plan_->publish(stopped);
        published_escape_plan_ = false;
      }
      return output;
    }
    resetEscape();
    published_escape_plan_ = false;
    auto command = base_->computeVelocityCommands(pose, velocity, goal_checker);
    limitInwardApproach(command.twist, current, tf2::getYaw(pose.pose.orientation));
    return command;
  }

private:
  void limitInwardApproach(geometry_msgs::msg::Twist & twist, escape::Point current, double yaw)
  {
    const auto speed = std::hypot(twist.linear.x, twist.linear.y);
    if (speed <= 0.0) {
      return;
    }
    const auto angle = yaw + std::atan2(twist.linear.y, twist.linear.x);
    constexpr double reaction_seconds = 0.15;
    const auto braking = speed * reaction_seconds + speed * speed / (2.0 * braking_rate_);
    auto * grid = costmap_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
    const auto step = grid->getResolution() * 0.5;
    const int count = std::max(1, static_cast<int>(std::ceil(braking / step)));
    for (int index = 1; index <= count; ++index) {
      const auto ahead = braking * index / count;
      const auto point = escape::along(current, angle, ahead);
      if (escape::clearance(*grid, point, escape::normal_clearance_m) >=
        escape::normal_clearance_m && !escape::inscribed(*grid, point))
      {
        continue;
      }
      const auto available = braking * (index - 1) / count;
      const auto reaction_speed = braking_rate_ * reaction_seconds;
      const auto permitted = std::sqrt(reaction_speed * reaction_speed +
        2.0 * braking_rate_ * available) - reaction_speed;
      twist.linear.x *= permitted / speed;
      twist.linear.y *= permitted / speed;
      return;
    }
  }

  void resetEscape()
  {
    escaping_ = false;
    still_since_.reset();
  }

  void updateSpeed(escape::Point point, double stamp)
  {
    if (previous_pose_ && stamp > previous_stamp_) {
      estimated_speed_ = std::hypot(point.x - previous_pose_->x, point.y - previous_pose_->y) /
        (stamp - previous_stamp_);
    } else if (!previous_pose_ || stamp < previous_stamp_) {
      estimated_speed_ = std::numeric_limits<double>::infinity();
      still_since_.reset();
    }
    if (!previous_pose_ || stamp != previous_stamp_) {
      previous_pose_ = point;
      previous_stamp_ = stamp;
    }
  }

  geometry_msgs::msg::TwistStamped escapeCommand(
    const geometry_msgs::msg::PoseStamped & pose, escape::Point current, double stamp)
  {
    geometry_msgs::msg::TwistStamped output;
    output.header.stamp = clock_->now();
    const auto age = clock_->now().seconds() - stamp;
    if (!costmap_->isCurrent() || plan_.poses.empty() || age < -.1 || age > .25)
    {
      resetEscape();
      return output;
    }
    if (!escaping_) {
      if (estimated_speed_ > .04) {
        still_since_.reset();
        return output;
      }
      if (!still_since_) {
        still_since_ = stamp;
      }
      if (stamp - *still_since_ < .25) {
        return output;
      }
    }
    geometry_msgs::msg::PoseStamped target;
    try {
      auto goal = plan_.poses.back();
      goal.header.frame_id = plan_.header.frame_id;
      goal.header.stamp = rclcpp::Time(0);
      target = tf_->transform(goal, pose.header.frame_id);
    } catch (const tf2::TransformException &) {
      resetEscape();
      return output;
    }

    auto * grid = costmap_->getCostmap();
    std::optional<escape::Point> endpoint;
    {
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
      endpoint = escape::find(*grid, current, {target.pose.position.x, target.pose.position.y}, radius_);
    }
    if (!endpoint) {
      resetEscape();
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 1000,
        "C072 inflated position has no outward segment clear of occupied and unknown cells");
      return output;
    }
    const auto speed = std::min(escape::escape_speed_mps, speed_limit_.load());
    if (speed <= 0.0) {
      return output;
    }
    const auto angle = std::atan2(endpoint->y - current.y, endpoint->x - current.x);
    const auto yaw = tf2::getYaw(pose.pose.orientation);
    if (!std::isfinite(yaw)) {
      resetEscape();
      return output;
    }
    const auto length = std::hypot(endpoint->x - current.x, endpoint->y - current.y);
    dwb_msgs::msg::Trajectory2D preview;
    for (int index = 0; index <= 30; ++index) {
      const auto t = horizon_ * index / 30.0;
      const auto point = escape::along(current, angle, std::min(length, speed * t));
      geometry_msgs::msg::Pose2D sample;
      sample.x = point.x;
      sample.y = point.y;
      sample.theta = yaw;
      preview.poses.push_back(sample);
      preview.time_offsets.push_back(rclcpp::Duration::from_seconds(t));
    }
    prediction_->prepare(preview.poses.front(), nav_2d_msgs::msg::Twist2D{},
      preview.poses.back(), nav_2d_msgs::msg::Path2D{});
    try {
      prediction_->scoreTrajectory(preview);
    } catch (const dwb_core::IllegalTrajectoryException &) {
      resetEscape();
      return output;
    }
    if (!escaping_) {
      RCLCPP_INFO(logger_, "C072 leaving inner inflation at %.2f m/s after stopped clearance check", speed);
    }
    // Keep the same velocity gate contract as an ordinary DWB command.
    // This is the just-checked path, in the controller frame at the pose stamp.
    nav_msgs::msg::Path path;
    path.header = pose.header;
    path.poses.reserve(preview.poses.size());
    for (const auto & sample : preview.poses) {
      geometry_msgs::msg::PoseStamped point = pose;
      point.pose.position.x = sample.x;
      point.pose.position.y = sample.y;
      path.poses.push_back(std::move(point));
    }
    escape_plan_->publish(path);
    published_escape_plan_ = true;
    escaping_ = true;
    output.twist.linear.x = speed * std::cos(angle - yaw);
    output.twist.linear.y = speed * std::sin(angle - yaw);
    return output;
  }

  pluginlib::ClassLoader<nav2_core::Controller> controllers_{"nav2_core", "nav2_core::Controller"};
  pluginlib::ClassLoader<dwb_core::TrajectoryCritic> critics_{"dwb_core", "dwb_core::TrajectoryCritic"};
  nav2_core::Controller::Ptr base_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr escape_plan_;
  dwb_core::TrajectoryCritic::Ptr prediction_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{rclcpp::get_logger("InflationAwareController")};
  nav_msgs::msg::Path plan_;
  double radius_{escape::padded_radius_m};
  double horizon_{0.0};
  double braking_rate_{0.0};
  double nominal_speed_{0.0};
  std::atomic<double> speed_limit_{0.0};
  bool escaping_{false};
  bool published_escape_plan_{false};
  std::optional<double> still_since_;
  std::optional<escape::Point> previous_pose_;
  double previous_stamp_{0.0};
  double estimated_speed_{std::numeric_limits<double>::infinity()};
};
}  // namespace rmul_c072_planner

PLUGINLIB_EXPORT_CLASS(rmul_c072_planner::InflationAwareController, nav2_core::Controller)
