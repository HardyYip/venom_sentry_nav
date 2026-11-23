// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

#include <dwb_plugins/standard_traj_generator.hpp>
#include <dwb_plugins/xy_theta_iterator.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <stdexcept>

namespace rmul_c072_planner
{
class LowSpeedIterator : public dwb_plugins::XYThetaIterator
{
public:
  void startNewIteration(const nav_2d_msgs::msg::Twist2D & current, double horizon) override
  {
    XYThetaIterator::startNewIteration(current, horizon);
    count_ = 0;
    index_ = 0;
    auto limits = kinematics_handler_->getKinematics();
    if (std::hypot(current.x, current.y) > 0.28 ||
      limits.getMinTheta() != 0.0 || limits.getMaxTheta() != 0.0)
    {
      return;
    }

    const auto bounded_x = std::clamp(current.x, limits.getMinX(), limits.getMaxX());
    const auto bounded_y = std::clamp(current.y, limits.getMinY(), limits.getMaxY());
    using dwb_plugins::projectVelocity;
    const auto min_x = projectVelocity(
      bounded_x, limits.getAccX(), limits.getDecelX(), horizon, limits.getMinX());
    const auto max_x = projectVelocity(
      bounded_x, limits.getAccX(), limits.getDecelX(), horizon, limits.getMaxX());
    const auto min_y = projectVelocity(
      bounded_y, limits.getAccY(), limits.getDecelY(), horizon, limits.getMinY());
    const auto max_y = projectVelocity(
      bounded_y, limits.getAccY(), limits.getDecelY(), horizon, limits.getMaxY());

    // At crawl speed the full-range Cartesian grid mostly offers its
    // axes or diagonals. Include intermediate headings in the same motion
    // window so DWB can closely follow the available corridor.
    for (const auto speed : speeds_) {
      for (const auto & unit : headings_) {
        const auto x = speed * unit.x;
        const auto y = speed * unit.y;
        if (x >= min_x && x <= max_x && y >= min_y && y <= max_y && isValidSpeed(x, y, 0.0)) {
          auto & sample = extra_[count_++];
          sample.x = x;
          sample.y = y;
          sample.theta = 0.0;
        }
      }
    }
  }

  bool hasMoreTwists() override
  {
    return index_ < count_ || XYThetaIterator::hasMoreTwists();
  }

  nav_2d_msgs::msg::Twist2D nextTwist() override
  {
    if (index_ < count_) {
      return extra_[index_++];
    }
    return XYThetaIterator::nextTwist();
  }

private:
  struct Heading {double x; double y;};
  static constexpr std::array<double, 4> speeds_{0.08, 0.16, 0.24, 0.32};
  static constexpr std::size_t sectors_ = 32;
  const std::array<Heading, sectors_> headings_ = [] {
      std::array<Heading, sectors_> headings{};
      for (std::size_t sector = 0; sector < sectors_; ++sector) {
        const double angle = 2.0 * std::acos(-1.0) * sector / sectors_;
        headings[sector] = {std::cos(angle), std::sin(angle)};
      }
      return headings;
    }();
  std::array<nav_2d_msgs::msg::Twist2D, speeds_.size() * sectors_> extra_;
  std::size_t count_{0}, index_{0};
};

class FineLowSpeedTrajectoryGenerator : public dwb_plugins::StandardTrajectoryGenerator
{
public:
  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & node, const std::string & name) override
  {
    StandardTrajectoryGenerator::initialize(node, name);
    const auto parameter = name + ".max_accel_xy";
    if (!node->has_parameter(parameter)) {
      node->declare_parameter(parameter, -1.0);
    }
    max_accel_xy_ = node->get_parameter(parameter).as_double();
    if (!std::isfinite(max_accel_xy_)) {
      throw std::invalid_argument("The planar acceleration limit must be finite");
    }
  }

protected:
  nav_2d_msgs::msg::Twist2D computeNewVelocity(
    const nav_2d_msgs::msg::Twist2D & desired, const nav_2d_msgs::msg::Twist2D & current,
    double dt) override
  {
    auto result = StandardTrajectoryGenerator::computeNewVelocity(desired, current, dt);
    if (max_accel_xy_ >= 0.0) {
      const double dx = result.x - current.x;
      const double dy = result.y - current.y;
      const double delta = std::hypot(dx, dy);
      const double allowed = max_accel_xy_ * dt;
      if (delta > allowed) {
        const double factor = allowed / delta;
        result.x = current.x + factor * dx;
        result.y = current.y + factor * dy;
      }
    }
    return result;
  }

  void initializeIterator(const nav2_util::LifecycleNode::SharedPtr & node) override
  {
    velocity_iterator_ = std::make_shared<LowSpeedIterator>();
    velocity_iterator_->initialize(node, kinematics_handler_, plugin_name_);
  }

private:
  double max_accel_xy_{-1.0};
};
}  // namespace rmul_c072_planner

PLUGINLIB_EXPORT_CLASS(rmul_c072_planner::FineLowSpeedTrajectoryGenerator, dwb_core::TrajectoryGenerator)
