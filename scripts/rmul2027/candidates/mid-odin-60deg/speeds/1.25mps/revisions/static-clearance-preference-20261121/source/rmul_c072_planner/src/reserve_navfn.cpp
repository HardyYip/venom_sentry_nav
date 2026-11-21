// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nav2_core/exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_navfn_planner/navfn_planner.hpp>
#include <opencv2/imgproc.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "observed_motion.hpp"
#include "rmul_c072_planner/msg/observed_obstacles.hpp"

namespace rmul_c072_planner
{
using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using pb_omni_pid_pursuit_controller::observed_motion::Point;
using pb_omni_pid_pursuit_controller::observed_motion::Tracker;
using Cloud = sensor_msgs::msg::PointCloud2;
using Pose = geometry_msgs::msg::PoseStamped;

class ReserveNavfn : public nav2_navfn_planner::NavfnPlanner
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    NavfnPlanner::configure(parent, name, tf, map);
    auto node = parent.lock();
    if (!node) {
      throw std::runtime_error("Missing planner node");
    }
    const auto horizon_parameter = name + ".moving_route_prediction_seconds";
    if (!node->has_parameter(horizon_parameter)) {
      node->declare_parameter(horizon_parameter, 2.25);
    }
    moving_route_prediction_seconds_ = node->get_parameter(horizon_parameter).as_double();
    if (!std::isfinite(moving_route_prediction_seconds_) ||
      moving_route_prediction_seconds_ < 0.5 || moving_route_prediction_seconds_ > 4.0)
    {
      throw std::invalid_argument("moving_route_prediction_seconds must be in [0.5, 4.0]");
    }
    const auto padding_parameter = name + ".static_route_preference_m";
    if (!node->has_parameter(padding_parameter)) {
      node->declare_parameter(padding_parameter, 0.);
    }
    static_route_preference_ = node->get_parameter(padding_parameter).as_double();
    if (!std::isfinite(static_route_preference_) ||
      static_route_preference_ < 0. || static_route_preference_ > 0.25)
    {
      throw std::invalid_argument("static_route_preference_m must be in [0, 0.25]");
    }
    observations_ = node->create_publisher<msg::ObservedObstacles>("c072/observed_obstacles", rclcpp::SensorDataQoS());
    background_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> locked(tracks_mutex_);
        if (msg->info.resolution > 0.0 && msg->data.size() ==
          static_cast<std::size_t>(msg->info.width) * msg->info.height)
        {
          background_ = msg;
        }
      });
    const std::array<std::string, 2> sources{"mid/geometric_obstacles", "odin/obstacles"};
    for (std::size_t i = 0; i < sources.size(); ++i) {
      cloud_subs_[i] = node->create_subscription<Cloud>(
        sources[i], rclcpp::SensorDataQoS(), [this, i](Cloud::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> locked(tracks_mutex_);
          pending_[i].push_back(msg);
          if (pending_[i].size() > 4) {
            pending_[i].pop_front();
          }
        });
    }
    timer_ = node->create_wall_timer(std::chrono::milliseconds(40), [this]() {processClouds();});
  }

  void activate() override
  {
    NavfnPlanner::activate();
    observations_->on_activate();
  }

  void deactivate() override
  {
    observations_->on_deactivate();
    NavfnPlanner::deactivate();
  }

  void cleanup() override
  {
    timer_.reset();
    background_sub_.reset();
    for (auto & sub : cloud_subs_) {sub.reset();}
    {
      std::lock_guard<std::mutex> locked(tracks_mutex_);
      background_.reset();
      observations_.reset();
      for (auto & tracker : trackers_) {tracker.clear();}
      for (auto & queue : pending_) {queue.clear();}
    }
    previous_path_.poses.clear();
    NavfnPlanner::cleanup();
  }

  nav_msgs::msg::Path createPlan(const Pose & start, const Pose & goal) override
  {
    std::lock_guard<std::mutex> serial(plan_mutex_);
    Costmap2D original;
    {
      std::lock_guard<Costmap2D::mutex_t> locked(*costmap_->getMutex());
      original = *costmap_;
    }
    const auto resolution = original.getResolution();
    if (resolution <= 0.0 || original.getSizeInCellsX() < 2 || original.getSizeInCellsY() < 2) {
      throw nav2_core::PlannerException("Global costmap missing");
    }
    Costmap2D preferred(original);
    const bool moving = addPredictions(preferred);
    const double margin = .50 + resolution / std::sqrt(2.);
    struct RestoreMap
    {
      Costmap2D * & slot;
      Costmap2D * original;
      ~RestoreMap() {slot = original;}
    } guard{costmap_, costmap_};
    const auto blocked = lethalMask(original);
    cv::Mat distance;
    cv::distanceTransform(blocked, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    if (!moving && static_route_preference_ > 0.) {
      preferStaticClearance(preferred, distance, margin);
    }


    // Use the C072 planner and its existing tolerance/orientation parameters.
    // Only the private planning snapshot is changed.
    for (double extra : {resolution / 2., 0.}) {
      Costmap2D planning_grid(preferred);
      for (unsigned int y = 0; y < original.getSizeInCellsY(); ++y) {
        for (unsigned int x = 0; x < original.getSizeInCellsX(); ++x) {
          if (distance.at<float>(y, x) * resolution <= margin + extra) {
            planning_grid.setCost(x, y, LETHAL_OBSTACLE);
          }
        }
      }
      costmap_ = &planning_grid;
      try {
        auto path = NavfnPlanner::createPlan(start, goal);
        if (validPath(original, start, path, margin)) {
          RCLCPP_INFO_THROTTLE(logger_, *clock_, 3000,
            "C072 clearance checked %zu global waypoints", path.poses.size());
          return stablePlan(original, start, goal, std::move(path), margin, moving);
        }
      } catch (const nav2_core::PlannerException &) {
        // Retry once with a wider grid for the interpolated NavFn segments.
      }
      costmap_ = &original;
    }
    // C072's local controller still enforces the independent hard reserve.
    // A stale or overly conservative global cell must not abort the entire
    // behavior tree before that controller can choose a safe local trajectory.
    costmap_ = &original;
    auto original_path = NavfnPlanner::createPlan(start, goal);
    if (!original_path.poses.empty()) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 3000,
        "C072 ordinary NavFn route used with mandatory local clearance");
    }
    return stablePlan(original, start, goal, std::move(original_path), margin, moving);
  }

private:
  std::mutex plan_mutex_, tracks_mutex_;
  nav_msgs::msg::Path previous_path_;
  Pose previous_goal_;
  double previous_selected_{-1.};
  double moving_route_prediction_seconds_{2.25};
  double static_route_preference_{0.};
  std::array<Tracker, 2> trackers_;
  std::array<std::deque<Cloud::ConstSharedPtr>, 2> pending_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr background_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr background_sub_;
  std::array<rclcpp::Subscription<Cloud>::SharedPtr, 2> cloud_subs_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<msg::ObservedObstacles>::SharedPtr observations_;

  static double distance(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  static Point departure(const nav_msgs::msg::Path & path, const geometry_msgs::msg::Point & start)
  {
    for (const auto & pose : path.poses) {
      const auto & p = pose.pose.position;
      if (distance(p, start) >= .65) {return {p.x - start.x, p.y - start.y};}
    }
    return {};
  }

  nav_msgs::msg::Path stablePlan(const Costmap2D & map, const Pose & start, const Pose & goal,
    nav_msgs::msg::Path candidate, double margin, bool moving)
  {
    const double now = clock_->now().seconds();
    // Goal preparation can observe the preview route for up to eight seconds.
    // Carry its corridor into the first navigation replan only while it is
    // still valid on this snapshot and no measured obstacle is moving.
    constexpr double preview_handoff_seconds = 9.;
    const auto & origin = start.pose.position;
    if (!moving && !previous_path_.poses.empty() && now >= previous_selected_ &&
      now - previous_selected_ <= preview_handoff_seconds &&
      previous_goal_.header.frame_id == goal.header.frame_id &&
      distance(previous_goal_.pose.position, goal.pose.position) <= .03)
    {
      const auto near = std::min_element(previous_path_.poses.begin(), previous_path_.poses.end(),
        [&origin](const Pose & a, const Pose & b) {
          return distance(a.pose.position, origin) < distance(b.pose.position, origin);
        });
      if (distance(near->pose.position, origin) <= .20) {
        nav_msgs::msg::Path retained;
        retained.header = previous_path_.header;
        retained.header.stamp = clock_->now();
        retained.poses.assign(near, previous_path_.poses.end());
        const auto old_direction = departure(retained, origin);
        const auto new_direction = departure(candidate, origin);
        const bool reversal =
          old_direction.x * new_direction.x + old_direction.y * new_direction.y < 0.;
        if ((candidate.poses.empty() || reversal) && validPath(map, start, retained, margin)) {
          retained.poses.back().pose.orientation = goal.pose.orientation;
          RCLCPP_INFO_THROTTLE(logger_, *clock_, 1500,
            "C072 retained revalidated corridor %.2f s after preview or replan",
            now - previous_selected_);
          return retained;
        }
      }
    }
    if (!candidate.poses.empty()) {
      previous_path_ = candidate;
      previous_goal_ = goal;
      previous_selected_ = now;
    }
    return candidate;
  }

  void preferStaticClearance(Costmap2D & map, const cv::Mat & distance, double margin) const
  {
    const double resolution = map.getResolution();
    for (int y = 0; y < distance.rows; ++y) {
      const auto * distances = distance.ptr<float>(y);
      for (int x = 0; x < distance.cols; ++x) {
        const auto existing = map.getCost(x, y);
        if (existing >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
          continue;
        }
        const double reserve = distances[x] * resolution - margin;
        if (reserve > static_route_preference_) {
          continue;
        }
        const double risk = 1. - std::max(0., reserve) / static_route_preference_;
        const auto extra_cost = static_cast<unsigned char>(244. * std::sqrt(risk));
        map.setCost(x, y, std::max(existing, extra_cost));
      }
    }
  }

  static cv::Mat lethalMask(const Costmap2D & costmap)
  {
    cv::Mat mask(costmap.getSizeInCellsY(), costmap.getSizeInCellsX(), CV_8UC1);
    for (int y = 0; y < mask.rows; ++y) {
      for (int x = 0; x < mask.cols; ++x) {
        mask.at<unsigned char>(y, x) = costmap.getCost(x, y) >= LETHAL_OBSTACLE ? 0 : 255;
      }
    }
    return mask;
  }

  static double clearance(const Costmap2D & map, double x, double y, double margin)
  {
    unsigned int mx, my;
    if (!map.worldToMap(x, y, mx, my)) {return 0.;}
    const auto radius = static_cast<int>(std::ceil(margin / map.getResolution())) + 1;
    double minimum = std::numeric_limits<double>::infinity();
    for (int gy = std::max(0, static_cast<int>(my) - radius);
      gy <= std::min(static_cast<int>(map.getSizeInCellsY()) - 1, static_cast<int>(my) + radius); ++gy)
    {
      for (int gx = std::max(0, static_cast<int>(mx) - radius);
        gx <= std::min(static_cast<int>(map.getSizeInCellsX()) - 1, static_cast<int>(mx) + radius); ++gx)
      {
        if (map.getCost(gx, gy) < LETHAL_OBSTACLE) {continue;}
        double wx, wy;
        map.mapToWorld(gx, gy, wx, wy);
        minimum = std::min(minimum, std::hypot(x - wx, y - wy));
      }
    }
    return minimum;
  }

  static bool validPath(const Costmap2D & map, const Pose & start, const nav_msgs::msg::Path & path,
    double margin)
  {
    if (path.poses.empty()) {return false;}
    auto from = start.pose.position;
    auto previous = clearance(map, from.x, from.y, margin);
    if (previous <= .50) {return false;}
    for (const auto & pose : path.poses) {
      const auto & to = pose.pose.position;
      const double dx = to.x - from.x, dy = to.y - from.y;
      if (!std::isfinite(dx) || !std::isfinite(dy)) {return false;}
      const auto count = std::max(1, static_cast<int>(std::ceil(
        std::hypot(dx, dy) / (map.getResolution() * .5))));
      for (int i = 1; i <= count; ++i) {
        const auto pad = clearance(map, from.x + dx * i / count, from.y + dy * i / count, margin);
        // C072 allows an outward-only exit when already inside the grid reserve.
        if (pad <= .50 || (pad <= margin && (previous > margin || pad + 1e-8 < previous))) {
          return false;
        }
        previous = pad;
      }
      from = to;
    }
    return previous > margin;
  }

  bool nearStatic(double x, double y) const
  {
    const auto & g = *background_;
    const auto & q = g.info.origin.orientation;
    const double yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.z * q.z + q.y * q.y));
    const double dx = x - g.info.origin.position.x, dy = y - g.info.origin.position.y;
    const int mx = std::floor((std::cos(yaw) * dx + std::sin(yaw) * dy) / g.info.resolution);
    const int my = std::floor((-std::sin(yaw) * dx + std::cos(yaw) * dy) / g.info.resolution);
    if (mx < 0 || my < 0 || mx >= static_cast<int>(g.info.width) || my >= static_cast<int>(g.info.height)) {
      return true;
    }
    for (int j = std::max(0, my - 2); j <= std::min(static_cast<int>(g.info.height) - 1, my + 2); ++j) {
      for (int i = std::max(0, mx - 2); i <= std::min(static_cast<int>(g.info.width) - 1, mx + 2); ++i) {
        if (g.data[j * g.info.width + i] >= 65) {return true;}
      }
    }
    return false;
  }

  void processClouds()
  {
    std::lock_guard<std::mutex> lock(tracks_mutex_);
    if (!background_ || background_->header.frame_id != global_frame_) {return;}
    const double now = clock_->now().seconds();
    for (std::size_t i = 0; i < trackers_.size(); ++i) {
      auto & queue = pending_[i];
      while (!queue.empty()) {
        const auto msg = queue.front();
        const auto time = rclcpp::Time(msg->header.stamp);
        const auto age = now - time.seconds();
        if (age > .5 || age < -.1) {queue.pop_front();continue;}
        tf2::Transform transform;
        try {
          tf2::fromMsg(tf_->lookupTransform(global_frame_, msg->header.frame_id, time).transform,
            transform);
        } catch (const tf2::TransformException &) {
          break;  // TF may arrive on the next spin.
        }
        queue.pop_front();
        if (msg->is_bigendian || msg->height != 1 || msg->point_step != 16 ||
          msg->row_step != msg->width * 16 || msg->data.size() != msg->row_step)
        {
          continue;
        }
        std::vector<Point> points;
        std::set<std::pair<int, int>> cells;
        try {
          if (msg->width) {
            sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
            for (; x != x.end() && points.size() < 1024; ++x, ++y, ++z) {
              if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {continue;}
              const auto p = transform * tf2::Vector3(*x, *y, *z);
              if (p.z() < .06 || p.z() > 2. || nearStatic(p.x(), p.y())) {continue;}
              if (cells.emplace(std::floor(p.x() / .025), std::floor(p.y() / .025)).second) {
                points.push_back({p.x(), p.y()});
              }
            }
          }
        } catch (const std::runtime_error &) {
          continue;
        }
        trackers_[i].update(time.seconds(), Tracker::clusters(points));
      }
    }
    auto observations = observations_;
    if (!observations || !observations->is_activated()) {return;}
    msg::ObservedObstacles update;
    update.header.stamp = clock_->now();
    update.header.frame_id = global_frame_;
    for (const auto & tracker : trackers_) {
      for (const auto & track : tracker.active(now)) {
        if (now - track.stamp > .5) {continue;}
        msg::ObservedObstacle sample;
        sample.stamp = rclcpp::Time(static_cast<int64_t>(std::llround(track.stamp * 1.e9)));
        sample.center.x = track.center.x;
        sample.center.y = track.center.y;
        sample.velocity.x = track.velocity.x;
        sample.velocity.y = track.velocity.y;
        sample.radius = track.radius;
        update.obstacles.push_back(sample);
      }
    }
    observations->publish(update);
  }

  bool addPredictions(Costmap2D & grid)
  {
    std::lock_guard<std::mutex> lock(tracks_mutex_);
    const double now = clock_->now().seconds(), res = grid.getResolution();
    unsigned int count = 0;
    for (const auto & tracker : trackers_) {
      for (const auto & track : tracker.active(now)) {
        const double age = std::max(0., now - track.stamp);
        if (age > .5) {continue;}
        ++count;
        for (double t = age; t <= age + moving_route_prediction_seconds_; t += .05) {
          const double x = track.center.x + track.velocity.x * t;
          const double y = track.center.y + track.velocity.y * t;
          const double radius = std::max(.15, track.radius) + .05 + .15 * t;
          unsigned int mx, my;
          if (!grid.worldToMap(x, y, mx, my)) {continue;}
          const int extent = std::ceil(radius/res);
          for (int j = std::max(0, static_cast<int>(my) - extent);
            j <= std::min(static_cast<int>(grid.getSizeInCellsY()) - 1, static_cast<int>(my) + extent); ++j)
          {
            for (int i = std::max(0, static_cast<int>(mx) - extent);
              i <= std::min(static_cast<int>(grid.getSizeInCellsX()) - 1, static_cast<int>(mx) + extent); ++i)
            {
              double wx, wy;
              grid.mapToWorld(i, j, wx, wy);
              if (std::hypot(x - wx, y - wy) <= radius &&
                grid.getCost(i, j) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
              {
                // A motion hypothesis shapes the route. Only actual costmap
                // occupancy participates in the independent clearance check.
                grid.setCost(i, j, std::max<unsigned char>(grid.getCost(i, j), 250));
              }
            }
          }
        }
      }
    }
    if (count) {
      RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000,
        "Planning around %u observed moving tracks with %.2f s soft prediction",
        count, moving_route_prediction_seconds_);
    }
    return count > 0;
  }
};
}  // namespace rmul_c072_planner

PLUGINLIB_EXPORT_CLASS(rmul_c072_planner::ReserveNavfn, nav2_core::GlobalPlanner)
