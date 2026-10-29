// Copyright 2026 HardyYip contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <iostream>
#include <nav2_costmap_2d/footprint.hpp>
#include <stdexcept>
int main() {
  // Exact existing C052 global polygon, including its stored precision.
  const double coordinates[][2] = {
      {0.434978334, 0.0},           {0.426620347, 0.084860063},
      {0.401867579, 0.166459002},   {0.361671266, 0.241661014},
      {0.307576129, 0.307576129},   {0.241661014, 0.361671266},
      {0.166459002, 0.401867579},   {0.084860063, 0.426620347},
      {0.0, 0.434978334},           {-0.084860063, 0.426620347},
      {-0.166459002, 0.401867579},  {-0.241661014, 0.361671266},
      {-0.307576129, 0.307576129},  {-0.361671266, 0.241661014},
      {-0.401867579, 0.166459002},  {-0.426620347, 0.084860063},
      {-0.434978334, 0.0},          {-0.426620347, -0.084860063},
      {-0.401867579, -0.166459002}, {-0.361671266, -0.241661014},
      {-0.307576129, -0.307576129}, {-0.241661014, -0.361671266},
      {-0.166459002, -0.401867579}, {-0.084860063, -0.426620347},
      {-0.0, -0.434978334},         {0.084860063, -0.426620347},
      {0.166459002, -0.401867579},  {0.241661014, -0.361671266},
      {0.307576129, -0.307576129},  {0.361671266, -0.241661014},
      {0.401867579, -0.166459002},  {0.426620347, -0.084860063},
  };
  std::vector<geometry_msgs::msg::Point> raw;
  for (const auto &coordinate : coordinates) {
    geometry_msgs::msg::Point p;
    p.x = coordinate[0];
    p.y = coordinate[1];
    raw.push_back(p);
  }
  auto local = raw;
  nav2_costmap_2d::padFootprint(local, .02);
  double li, lc;
  nav2_costmap_2d::calculateMinAndMaxDistances(local, li, lc);
  const double required = lc + std::sqrt(2.) * .05 / 2.;
  int checks = 0;
  for (double pad : {.02, .07}) {
    auto global = raw;
    nav2_costmap_2d::padFootprint(global, pad);
    double gi, gc;
    nav2_costmap_2d::calculateMinAndMaxDistances(global, gi, gc);
    bool sufficient = gi >= required;
    if (sufficient != (pad == .07))
      throw std::runtime_error("Global inscribed planning clearance does not "
                               "cover local footprint plus cell half diagonal");
    ++checks;
    std::cout << "{\"global_padding\":" << pad << ",\"global_inscribed\":" << gi
              << ",\"local_circumscribed\":" << lc
              << ",\"required_with_half_cell_diagonal\":" << required
              << ",\"sufficient\":" << (sufficient ? "true" : "false")
              << ",\"passed\":true}" << std::endl;
  }
  std::cout << "{\"passed\":true,\"checks\":" << checks << "}" << std::endl;
}
