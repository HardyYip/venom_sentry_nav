// Test fixture for three ordinary waypoints on the disconnected navigation cart.
#include "rmul2027_decision/core.hpp"
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc != 2) return 1;
  std::ifstream points(argv[1]);
  rmul2027_decision::PolicyConfig config;
  double x, y;
  while (points >> x >> y) config.points.push_back({"", x, y, 0});
  if (config.points.size() != 3) return 2;
  rmul2027_decision::Policy policy(config);
  rmul2027_decision::Input nominal;
  nominal.healthy = true;
  std::string event;
  double clock;
  while (std::cin >> event >> clock) {
    if (event == "arrived") policy.arrival(policy.task(), clock);
    else if (event != "tick") return 3;
    policy.update(nominal, clock);
    const auto goal = policy.task();
    std::cout << goal.key << " " << goal.x << " " << goal.y << std::endl;
  }
}
