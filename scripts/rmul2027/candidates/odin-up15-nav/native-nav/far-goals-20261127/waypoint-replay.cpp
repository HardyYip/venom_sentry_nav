#include "rmul2027_decision/core.hpp"
#include <iostream>
#include <vector>

// Replay successful navigation receipts into the stock waypoint scheduler.
int main() {
  using namespace rmul2027_decision;
  PolicyConfig config;
  std::vector<double> durations;
  double x, y, duration;
  while (std::cin >> x >> y >> duration) {
    config.points.push_back({"", x, y, 0});
    durations.push_back(duration);
  }
  if (durations.size() != 5) return 1;
  Policy policy(config);
  Input input;
  input.healthy = true;
  double time = 0;
  for (size_t i = 0; i < durations.size(); ++i) {
    policy.update(input, time);
    const Goal sent = policy.task();
    if (sent.x != config.points[i].x || sent.y != config.points[i].y) return 2;
    time += durations[i];
    policy.update(input, time);
    if (policy.task().key != sent.key) return 3;
    policy.arrival(sent, time);
    policy.update(input, time + config.dwell - 0.001);
    if (policy.task().key != sent.key) return 4;
    time += config.dwell + 0.001;
    policy.update(input, time);
    const auto &expected = config.points[(i+1) % durations.size()];
    const Goal next = policy.task();
    if (next.x != expected.x || next.y != expected.y || next.key == sent.key) return 5;
    std::cout << sent.key << " -> " << next.key << '\n';
  }
}
