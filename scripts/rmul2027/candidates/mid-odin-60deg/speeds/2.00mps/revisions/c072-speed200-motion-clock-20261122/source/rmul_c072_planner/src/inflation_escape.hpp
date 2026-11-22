// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace rmul_c072_planner::inflation_escape
{
struct Point
{
  double x;
  double y;
};

// Conservative circle enclosing the 0.58 m by 0.50 m chassis and 2 cm padding.
inline constexpr double padded_radius_m = 0.4111;
inline constexpr double normal_clearance_m = 0.50;
inline constexpr double escape_end_clearance_m = 0.56;
inline constexpr double max_escape_m = 0.70;
inline constexpr double escape_speed_mps = 0.20;

inline Point along(Point from, double angle, double distance)
{
  return {from.x + std::cos(angle) * distance, from.y + std::sin(angle) * distance};
}

inline double clearance(const nav2_costmap_2d::Costmap2D & grid, Point p, double limit)
{
  unsigned int mx, my;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
    !grid.worldToMap(p.x, p.y, mx, my))
  {
    return 0.0;
  }
  const auto resolution = grid.getResolution();
  const auto half = resolution / 2.0;
  const auto width = static_cast<int>(grid.getSizeInCellsX());
  const auto height = static_cast<int>(grid.getSizeInCellsY());
  double nearest = std::min({limit, p.x - grid.getOriginX(), p.y - grid.getOriginY(),
    grid.getOriginX() + width * resolution - p.x, grid.getOriginY() + height * resolution - p.y});
  const int cells = static_cast<int>(std::ceil((nearest + half) / resolution)) + 1;
  for (int y = std::max(0, static_cast<int>(my) - cells);
    y <= std::min(height - 1, static_cast<int>(my) + cells); ++y)
  {
    for (int x = std::max(0, static_cast<int>(mx) - cells);
      x <= std::min(width - 1, static_cast<int>(mx) + cells); ++x)
    {
      if (grid.getCost(x, y) < nav2_costmap_2d::LETHAL_OBSTACLE) {
        continue;
      }
      double wx, wy;
      grid.mapToWorld(x, y, wx, wy);
      // Contact is at the edge of the occupied cell, not its center.
      nearest = std::min(nearest, std::hypot(std::max(0.0, std::abs(p.x - wx) - half),
        std::max(0.0, std::abs(p.y - wy) - half)));
    }
  }
  return nearest;
}

inline bool inscribed(const nav2_costmap_2d::Costmap2D & grid, Point p)
{
  unsigned int x, y;
  return grid.worldToMap(p.x, p.y, x, y) &&
         grid.getCost(x, y) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
}

inline double freeBrakingDistance(
  const nav2_costmap_2d::Costmap2D & grid, Point current, double angle, double distance)
{
  const auto step = grid.getResolution() * 0.5;
  const int count = std::max(1, static_cast<int>(std::ceil(distance / step)));
  for (int index = 1; index <= count; ++index) {
    const auto point = along(current, angle, distance * index / count);
    if (clearance(grid, point, normal_clearance_m) < normal_clearance_m ||
      inscribed(grid, point))
    {
      return distance * (index - 1) / count;
    }
  }
  return distance;
}

inline std::optional<Point> find(
  const nav2_costmap_2d::Costmap2D & grid, Point start, Point goal, double radius)
{
  const double step = grid.getResolution() / 2.0;
  const double initial = clearance(grid, start, escape_end_clearance_m);
  if (initial <= radius) {
    return std::nullopt;
  }

  const double preferred = std::atan2(goal.y - start.y, goal.x - start.x);
  double best_score = max_escape_m + 0.04;
  std::optional<Point> best;
  for (int ray = 0; ray < 24; ++ray) {
    const auto angle = preferred + ray * (2.0 * std::acos(-1.0) / 24.0);
    auto previous = initial;
    for (double d = step; d <= max_escape_m; d += step) {
      const auto pt = along(start, angle, d);
      const auto next = clearance(grid, pt, escape_end_clearance_m);
      if (next <= radius || next + 1.e-6 < previous) {
        break;
      }
      if (next >= escape_end_clearance_m && !inscribed(grid, pt)) {
        // Check the small distance that may remain after the next zero command.
        const auto stop_margin = along(start, angle, d + 0.04);
        if (clearance(grid, stop_margin, normal_clearance_m) < normal_clearance_m ||
          inscribed(grid, stop_margin))
        {
          break;
        }
        const double score = d + 0.02 * (1.0 - std::cos(angle - preferred));
        if (score < best_score) {
          best = pt;
          best_score = score;
        }
        break;
      }
      previous = next;
    }
  }
  return best;
}
}  // namespace rmul_c072_planner::inflation_escape
