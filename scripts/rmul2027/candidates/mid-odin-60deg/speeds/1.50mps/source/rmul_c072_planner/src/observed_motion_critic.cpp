// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

#include <dwb_core/exceptions.hpp>
#include <dwb_core/trajectory_critic.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include "observed_motion.hpp"
#include "trajectory_timing.hpp"
#include "rmul_c072_planner/msg/observed_obstacles.hpp"

namespace rmul_c072_planner
{
using pb_omni_pid_pursuit_controller::observed_motion::Track;
using pb_omni_pid_pursuit_controller::observed_motion::predictedGap;

class PredictedObstacleCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    auto node = node_.lock();
    if (!node) {throw std::runtime_error("Missing C072 controller node");}
    const auto horizon_parameter = dwb_plugin_name_ + ".sim_time";
    if (!node->has_parameter(horizon_parameter)) {node->declare_parameter(horizon_parameter, 1.7);}
    horizon_ = node->get_parameter(horizon_parameter).as_double();
    if (!std::isfinite(horizon_) || horizon_ <= 0.) {throw std::runtime_error("Invalid trajectory horizon");}
    clock_ = node->get_clock();
    tf_ = costmap_ros_->getTfBuffer();
    auto inbox = inbox_;
    subscription_ = node->create_subscription<msg::ObservedObstacles>(
      "c072/observed_obstacles", rclcpp::SensorDataQoS(),
      [inbox](msg::ObservedObstacles::ConstSharedPtr update) {
        std::lock_guard<std::mutex> lock(inbox->mutex);
        inbox->update = update;
      });
    RCLCPP_INFO(node->get_logger(), "C072 measured obstacle prediction active in local scoring");
  }

  bool prepare(const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Twist2D &,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override
  {
    active_.clear();
    scoring_now_ = clock_->now().seconds();
    msg::ObservedObstacles::ConstSharedPtr update;
    {
      std::lock_guard<std::mutex> lock(inbox_->mutex);
      update = inbox_->update;
    }
    if (!update || update->obstacles.empty()) {return true;}
    const double age = scoring_now_ - rclcpp::Time(update->header.stamp).seconds();
    if (age < -.1 || age > .5) {return true;}
    tf2::Transform local;
    try {
      const auto t = tf_->lookupTransform(
        costmap_ros_->getGlobalFrameID(), update->header.frame_id, tf2::TimePointZero);
      const auto transform_age = scoring_now_ - rclcpp::Time(t.header.stamp).seconds();
      if (transform_age < -.1 || transform_age > .5) {return true;}
      tf2::fromMsg(t.transform, local);
    } catch (const tf2::TransformException &) {
      return true;
    }
    for (const auto & obstacle : update->obstacles) {
      const double stamp = rclcpp::Time(obstacle.stamp).seconds();
      const auto & p = obstacle.center;
      const auto & v = obstacle.velocity;
      if (scoring_now_ - stamp < -.1 || scoring_now_ - stamp > .5 ||
        !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(v.x) ||
        !std::isfinite(v.y) || !std::isfinite(obstacle.radius) || obstacle.radius <= 0.)
      {
        continue;
      }
      const auto center = local * tf2::Vector3(p.x, p.y, 0.);
      const auto velocity = local.getBasis() * tf2::Vector3(v.x, v.y, 0.);
      active_.push_back(Track{0, stamp, {center.x(), center.y()}, {velocity.x(), velocity.y()},
          obstacle.radius, true});
    }
    return true;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    if (active_.empty()) {return 0.;}
    double closest = .40, previous = 0.;
    for (std::size_t i = 0; i < trajectory.poses.size(); ++i) {
      const auto & point = trajectory.poses[i];
      const auto time = sampleTime(trajectory, i, horizon_);
      if (!time || *time < previous) {
        throw dwb_core::IllegalTrajectoryException(name_, "Invalid trajectory sample timing");
      }
      previous = *time;
      for (const auto & track : active_) {
        const double gap = predictedGap(track, {point.x, point.y}, scoring_now_, *time, .382883794 + .05);
        if (gap <= 0.) {
          throw dwb_core::IllegalTrajectoryException(name_, "Predicted observed moving obstacle collision");
        }
        closest = std::min(closest, gap);
      }
    }
    return .40 - closest;
  }

private:
  struct Inbox
  {
    std::mutex mutex;
    msg::ObservedObstacles::ConstSharedPtr update;
  };
  std::shared_ptr<Inbox> inbox_ = std::make_shared<Inbox>();
  rclcpp::Subscription<msg::ObservedObstacles>::SharedPtr subscription_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::vector<Track> active_;
  double scoring_now_{0.}, horizon_{0.};
};
}  // namespace rmul_c072_planner

PLUGINLIB_EXPORT_CLASS(rmul_c072_planner::PredictedObstacleCritic, dwb_core::TrajectoryCritic)
