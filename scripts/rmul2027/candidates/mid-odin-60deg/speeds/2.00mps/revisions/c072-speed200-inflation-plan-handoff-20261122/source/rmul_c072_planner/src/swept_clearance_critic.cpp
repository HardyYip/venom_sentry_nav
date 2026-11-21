#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>

#include "trajectory_timing.hpp"

#include <dwb_core/trajectory_critic.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "rmul_c072_planner/msg/observed_obstacles.hpp"

namespace rmul_c072_planner
{
class SweptClearanceCritic : public dwb_core::TrajectoryCritic
{
public:
  void onInit() override
  {
    const auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("Missing C072 controller node");
    }
    const auto parameter = dwb_plugin_name_ + ".sim_time";
    if (!node->has_parameter(parameter)) {
      node->declare_parameter(parameter, 1.5);
    }
    horizon_ = node->get_parameter(parameter).as_double();
    if (!std::isfinite(horizon_) || horizon_ <= 0.) {
      throw std::runtime_error("Invalid trajectory horizon");
    }
    const auto decel = node->get_parameter(dwb_plugin_name_ + ".decel_lim_x").as_double();
    const auto lateral_decel = node->get_parameter(dwb_plugin_name_ + ".decel_lim_y").as_double();
    braking_acceleration_ = std::min(-decel, -lateral_decel);
    if (!std::isfinite(braking_acceleration_) || braking_acceleration_ <= 0.) {
      throw std::runtime_error("A positive planar braking capability is required");
    }
    clock_ = node->get_clock();
    const auto inbox = inbox_;
    global_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
      "global_costmap/costmap_raw", rclcpp::QoS(1).transient_local(),
      [inbox](nav2_msgs::msg::Costmap::ConstSharedPtr grid) {
        std::lock_guard<std::mutex> lock(inbox->mutex);
        inbox->grid = std::move(grid);
        inbox->received = std::chrono::steady_clock::now();
      });
    motion_sub_ = node->create_subscription<msg::ObservedObstacles>(
      "c072/observed_obstacles", rclcpp::SensorDataQoS(),
      [inbox](msg::ObservedObstacles::ConstSharedPtr motion) {
        std::lock_guard<std::mutex> lock(inbox->mutex);
        inbox->motion = std::move(motion);
      });
  }

  bool prepare(
    const geometry_msgs::msg::Pose2D & pose, const nav_2d_msgs::msg::Twist2D & velocity,
    const geometry_msgs::msg::Pose2D &, const nav_2d_msgs::msg::Path2D &) override
  {
    map_ = costmap_ros_->getCostmap();
    std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map_->getMutex());
    if (map_->getSizeInCellsX() == 0 || map_->getSizeInCellsY() == 0) {
      return false;
    }
    resolution_ = map_->getResolution();
    swept_radius_ = 0.;
    for (const auto & corner : costmap_ros_->getRobotFootprint()) {
      swept_radius_ = std::max(swept_radius_, std::hypot(corner.x, corner.y));
    }
    // The base spins independently of the frame used to sample DWB trajectories.
    // Reserve the padded footprint's entire rotation and half a cell diagonal.
    swept_radius_ += resolution_ / std::sqrt(2.);
    cv::Mat grid(map_->getSizeInCellsY(), map_->getSizeInCellsX(), CV_8U, map_->getCharMap());
    cv::compare(grid, nav2_costmap_2d::LETHAL_OBSTACLE, free_cells_, cv::CMP_NE);
    cv::distanceTransform(free_cells_, distances_, cv::DIST_L2, cv::DIST_MASK_5);
    prepareGlobalMap(pose);
    current_speed_ = std::hypot(velocity.x, velocity.y);
    current_risk_ = proximityRisk(pose);
    return true;
  }

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D & trajectory) override
  {
    double peak_risk = current_risk_;
    // Include the braking time for the greater of observed and proposed speeds.
    const auto speed = std::max(current_speed_, std::hypot(trajectory.velocity.x, trajectory.velocity.y));
    const auto braking_horizon = std::min(horizon_, std::max(.65, speed / braking_acceleration_ + .15));
    for (std::size_t i = 0; i < trajectory.poses.size(); ++i) {
      const auto time = sampleTime(trajectory, i, horizon_);
      if (!time || *time > braking_horizon + 1.e-6) {
        break;
      }
      peak_risk = std::max(peak_risk, proximityRisk(trajectory.poses[i]));
    }
    // Paths that maintain or increase the current clearance carry no extra
    // cost, so the robot can still leave a narrow space.
    return peak_risk - current_risk_;
  }

private:
  using Costmap = nav2_msgs::msg::Costmap;
  struct Inbox
  {
    std::mutex mutex;
    Costmap::ConstSharedPtr grid;
    msg::ObservedObstacles::ConstSharedPtr motion;
    std::chrono::steady_clock::time_point received;
  };

  void prepareGlobalMap(const geometry_msgs::msg::Pose2D & pose)
  {
    global_ready_ = false;
    Costmap::ConstSharedPtr grid;
    msg::ObservedObstacles::ConstSharedPtr motion;
    {
      std::lock_guard<std::mutex> lock(inbox_->mutex);
      if (std::chrono::steady_clock::now() - inbox_->received > std::chrono::seconds(2)) {
        return;
      }
      grid = inbox_->grid;
      motion = inbox_->motion;
    }
    if (!grid || grid->header.frame_id.empty()) {
      return;
    }
    const auto & meta = grid->metadata;
    if (meta.size_x == 0 || meta.size_y == 0 || !std::isfinite(meta.resolution) ||
      meta.resolution <= 0. || grid->data.size() != static_cast<std::size_t>(meta.size_x) * meta.size_y)
    {
      return;
    }
    tf2::Transform from_local, origin;
    const auto now = clock_->now();
    try {
      auto stamped = costmap_ros_->getTfBuffer()->lookupTransform(
        grid->header.frame_id, costmap_ros_->getGlobalFrameID(), tf2::TimePointZero);
      const auto age = (now - rclcpp::Time(stamped.header.stamp)).seconds();
      if (age < -.1 || age > .30) {
        return;
      }
      tf2::fromMsg(stamped.transform, from_local);
      tf2::fromMsg(meta.origin, origin);
      grid_from_local_ = origin.inverse() * from_local;
    } catch (const tf2::TransformException &) {
      return;
    }
    // Only the extra global soft score yields to moving-object prediction.
    // Costmap collision checks and the ordinary local score stay active.
    const auto robot = from_local * tf2::Vector3(pose.x, pose.y, 0.);
    const auto current = now.seconds();
    if (current < previous_time_) {
      motion_until_ = 0.;
    }
    previous_time_ = current;
    if (motion && motion->header.frame_id == grid->header.frame_id) {
      for (const auto & track : motion->obstacles) {
        const auto age = (now - rclcpp::Time(track.stamp)).seconds();
        if (age >= -.1 && age <= .5 &&
          std::hypot(track.velocity.x, track.velocity.y) >= .15 &&
          std::hypot(track.center.x - robot.x(), track.center.y - robot.y()) <= 4.)
        {
          motion_until_ = current + 1.;
          break;
        }
      }
    }
    if (current < motion_until_) {
      if (auto node = node_.lock()) {
        RCLCPP_INFO_ONCE(node->get_logger(),
          "C072 swept clearance leaves global soft scoring to the moving-object planner");
      }
      return;
    }
    if (global_grid_ != grid) {
      cv::Mat cells = cv::Mat(grid->data, true).reshape(1, meta.size_y);
      cv::compare(cells, nav2_costmap_2d::LETHAL_OBSTACLE, global_free_cells_, cv::CMP_NE);
      cv::distanceTransform(global_free_cells_, global_distances_, cv::DIST_L2, cv::DIST_MASK_5);
      global_grid_ = std::move(grid);
    }
    global_ready_ = true;
    if (auto node = node_.lock()) {
      RCLCPP_INFO_ONCE(node->get_logger(), "C072 swept clearance uses aligned global occupied cells");
    }
  }

  double proximityRisk(const geometry_msgs::msg::Pose2D & pose) const
  {
    unsigned int x, y;
    double clearance = map_->worldToMap(pose.x, pose.y, x, y) ?
      distances_.at<float>(y, x) * resolution_ - swept_radius_ :
      preferred_clearance;
    if (global_ready_) {
      const auto point = grid_from_local_ * tf2::Vector3(pose.x, pose.y, 0.);
      const auto resolution = global_grid_->metadata.resolution;
      const double gx = point.x() / resolution, gy = point.y() / resolution;
      if (gx >= 0. && gy >= 0. && gx < global_distances_.cols && gy < global_distances_.rows) {
        const auto distance = global_distances_.at<float>(static_cast<int>(gy), static_cast<int>(gx));
        const auto radius = swept_radius_ + (resolution - resolution_) / std::sqrt(2.);
        clearance = std::min(clearance, distance * resolution - radius);
      }
    }
    return std::clamp(1. - clearance / preferred_clearance, 0., 1.);
  }

  static constexpr double preferred_clearance = .11;
  nav2_costmap_2d::Costmap2D * map_{nullptr};
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<Inbox> inbox_ = std::make_shared<Inbox>();
  rclcpp::Subscription<Costmap>::SharedPtr global_sub_;
  rclcpp::Subscription<msg::ObservedObstacles>::SharedPtr motion_sub_;
  Costmap::ConstSharedPtr global_grid_;
  tf2::Transform grid_from_local_;
  bool global_ready_{false};
  double previous_time_{0.};
  double motion_until_{0.};
  cv::Mat free_cells_;
  cv::Mat distances_;
  cv::Mat global_free_cells_;
  cv::Mat global_distances_;
  double current_risk_{0.};
  double current_speed_{0.};
  double braking_acceleration_{0.};
  double horizon_{0.};
  double resolution_{0.};
  double swept_radius_{0.};
};
}  // namespace rmul_c072_planner

PLUGINLIB_EXPORT_CLASS(rmul_c072_planner::SweptClearanceCritic, dwb_core::TrajectoryCritic)
