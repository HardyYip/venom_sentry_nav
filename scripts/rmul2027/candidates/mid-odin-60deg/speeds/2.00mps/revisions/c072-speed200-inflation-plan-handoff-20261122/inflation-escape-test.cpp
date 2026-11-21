// SPDX-License-Identifier: Apache-2.0
#include <cassert>
#include <cmath>
#include <iostream>

#include "inflation_escape.hpp"

int main()
{
  namespace escape = rmul_c072_planner::inflation_escape;
  namespace costs = nav2_costmap_2d;
  costs::Costmap2D map(100, 100, 0.05, -1.0, -2.5, costs::FREE_SPACE);
  for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y) {
    map.setCost(19, y, costs::LETHAL_OBSTACLE);
    for (unsigned int x = 20; x <= 28; ++x) {
      map.setCost(x, y, costs::INSCRIBED_INFLATED_OBSTACLE);
    }
  }
  const escape::Point inflated{0.43, 0.0};
  assert(escape::inscribed(map, inflated));
  const auto exit = escape::find(map, inflated, {-0.5, 0.0}, escape::padded_radius_m);
  assert(exit && exit->x > 0.55);
  assert(escape::clearance(map, *exit, 1.0) >= escape::escape_end_clearance_m);
  assert(!escape::find(map, {0.39, 0.0}, {1.0, 0.0}, escape::padded_radius_m));
  assert(!escape::find(map, {-2.0, 0.0}, {1.0, 0.0}, escape::padded_radius_m));

  for (unsigned int y = 0; y < map.getSizeInCellsY(); ++y) {
    map.setCost(38, y, costs::NO_INFORMATION);
  }
  assert(!escape::find(map, inflated, {1.5, 0.0}, escape::padded_radius_m));
  std::cout << "Inflated outward exit, inward preference, collision start, out of map "
    "and unknown-wall rejection passed\n";
}
