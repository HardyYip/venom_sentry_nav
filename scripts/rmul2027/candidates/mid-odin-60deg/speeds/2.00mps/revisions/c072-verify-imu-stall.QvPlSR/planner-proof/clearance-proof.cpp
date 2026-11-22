#include <chrono>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>
#include <vector>

#define inflation_escape prior_clearance
#include "../../c072-navfn-fresh-map-recovery.n6Du2L/source/rmul_c072_planner/src/inflation_escape.hpp"
#undef inflation_escape
#include "inflation_escape.hpp"

namespace previous = rmul_c072_planner::prior_clearance;
namespace revised = rmul_c072_planner::inflation_escape;

int main()
{
  nav2_costmap_2d::Costmap2D grid(160, 120, 0.05, -3.75, -2.65, 0);
  for (unsigned y = 0; y < 120; ++y) {
    grid.setCost(24, y, nav2_costmap_2d::LETHAL_OBSTACLE);
    grid.setCost(94, y, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    grid.setCost(110, y, nav2_costmap_2d::NO_INFORMATION);
  }
  for (unsigned x = 4; x < 50; ++x) {
    grid.setCost(x, 78, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
  struct Sample {double x, y, distance, angle, limit;};
  std::vector<Sample> samples;
  std::mt19937 random(42);
  const auto take = [&](double low, double high) {
      return std::uniform_real_distribution<double>(low, high)(random);
    };
  for (int i = 0; i < 30000; ++i) {
    samples.push_back({take(-4., 4.5), take(-3., 3.8), take(0., 1.5),
      take(-3.142, 3.142), take(.40, .65)});
  }
  for (const auto & s : samples) {
    if (previous::clearance(grid, {s.x, s.y}, s.limit) !=
      revised::clearance(grid, {s.x, s.y}, s.limit) ||
      previous::freeBrakingDistance(grid, {s.x, s.y}, s.angle, s.distance, s.limit) !=
      revised::freeBrakingDistance(grid, {s.x, s.y}, s.angle, s.distance, s.limit))
    {
      throw std::runtime_error("Clearance or braking distance changed");
    }
  }
  const auto measure = [&](auto distance) {
      const auto start = std::chrono::steady_clock::now();
      double sum = 0.;
      for (const auto & s : samples) {
        sum += distance(grid, {s.x, s.y}, s.angle, s.distance, s.limit);
      }
      const auto duration = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
      return std::pair<double, double>{sum, duration};
    };
  const auto [old_distance, old_ms] = measure(previous::freeBrakingDistance);
  const auto [new_distance, new_ms] = measure(revised::freeBrakingDistance);
  if (old_distance != new_distance) {
    throw std::runtime_error("Braking distance sum changed");
  }
  std::printf("PASS 30000 bit-identical clearance and braking probes. Prior %.2f ms, revised %.2f ms\n",
    old_ms, new_ms);
}
