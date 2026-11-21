// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>
#include <optional>
#include <dwb_msgs/msg/trajectory2_d.hpp>
#include <rclcpp/duration.hpp>

namespace rmul_c072_planner
{
inline std::optional<double> sampleTime(const dwb_msgs::msg::Trajectory2D & trajectory,
  std::size_t index, double horizon)
{
  const auto poses = trajectory.poses.size();
  const auto offsets = trajectory.time_offsets.size();
  if (offsets == 0 || index >= poses || (poses != offsets && poses != offsets + 1)) {
    return std::nullopt;
  }
  // Humble StandardTrajectoryGenerator pairs the offsets with the pose at
  // each step's start and may duplicate the final pose. The final index is
  // always the end of the simulation horizon.
  const double t = index < offsets ?
    rclcpp::Duration(trajectory.time_offsets[index]).seconds() : horizon;
  if (!std::isfinite(t) || t < 0. || t > horizon + 1.e-6) {return std::nullopt;}
  return t;
}
}  // namespace rmul_c072_planner
