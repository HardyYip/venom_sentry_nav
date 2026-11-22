#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_navfn_planner/navfn_planner.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "inflation_escape.hpp"

using geometry_msgs::msg::PoseStamped;
using nav2_costmap_2d::Costmap2D;
using namespace rmul_c072_planner::inflation_escape;

void require(bool ok, const std::string & message)
{
  if (!ok) {
    throw std::runtime_error(message);
  }
}

PoseStamped pose(double x, double y)
{
  PoseStamped value;
  value.header.frame_id = "map";
  value.pose.position.x = x;
  value.pose.position.y = y;
  value.pose.orientation.w = 1.0;
  return value;
}

void fill(Costmap2D & map, bool maze)
{
  constexpr int size = 120;
  map.resizeMap(size, size, 0.10, 0.0, 0.0);
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      const bool outside = x == 0 || y == 0 || x == size - 1 || y == size - 1;
      const bool divider = y > 0 && y % 15 == 0 && ((y / 15) % 2 ? x < 100 : x >= 20);
      map.setCost(x, y, outside || (maze && divider) ? 254 : 0);
    }
  }
}

void checkClearance(const Costmap2D & map, const nav_msgs::msg::Path & route)
{
  for (std::size_t i = 1; i < route.poses.size(); ++i) {
    const auto & a = route.poses[i - 1].pose.position;
    const auto & b = route.poses[i].pose.position;
    const int samples = std::max(1, static_cast<int>(std::ceil(std::hypot(b.x-a.x,b.y-a.y) / .025)));
    for (int sample = 0; sample <= samples; ++sample) {
      const double t = static_cast<double>(sample) / samples;
      require(clearance(map, {a.x+t*(b.x-a.x), a.y+t*(b.y-a.y)}, .51) >= .50,
        "invalid swept clearance on returned path");
    }
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("grid_proof");
  auto map_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("grid_proof_map");
  map_ros->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
  require(map_ros->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
    "costmap configure failed");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core", "nav2_core::GlobalPlanner");
  node->declare_parameter("Recovery.allow_unknown", false);
  node->declare_parameter("Recovery.tolerance", 0.15);
  node->declare_parameter("Recovery.allow_inflation_escape", true);
  node->declare_parameter("Native.allow_unknown", false);
  node->declare_parameter("Native.tolerance", 0.15);
  auto recovery = loader.createSharedInstance("rmul_c072_planner/ReserveNavfn");
  nav2_navfn_planner::NavfnPlanner native;
  recovery->configure(node, "Recovery", tf, map_ros);
  native.configure(node, "Native", tf, map_ros);
  auto & map = *map_ros->getCostmap();
  const auto from = pose(1.25, .75);
  const auto to = pose(10.75, 11.25);

  fill(map, true);
  const auto native_route = native.createPlan(from, to);
  require(native_route.poses.empty(), "fixture must exercise native NavFn failure");
  fill(map, true);
  const auto route = recovery->createPlan(from, to);
  require(route.poses.size() > 280, "reachable serpentine route was not recovered");
  require(route.poses.front().pose.position == from.pose.position, "recovered start differs");
  require(route.poses.back().pose.position == to.pose.position, "recovered goal differs");
  checkClearance(map, route);
  std::cout << "PASS NavFn no-path, verified grid route " << route.poses.size() << " points\n";

  for (const unsigned char wall : {254, 255}) {
    fill(map, true);
    for (int x = 0; x < 120; ++x) {
      map.setCost(x, 60, wall);
    }
    require(recovery->createPlan(from, to).poses.empty(), "blocked grid yielded a route");
    std::cout << "PASS complete barrier rejected, cost " << static_cast<int>(wall) << '\n';
  }
  fill(map, false);
  const auto ordinary = recovery->createPlan(from, to);
  require(!ordinary.poses.empty() && ordinary.poses.size() < route.poses.size(),
    "ordinary clear route failed");
  checkClearance(map, ordinary);
  std::cout << "PASS ordinary clear route " << ordinary.poses.size() << " points\n";
  recovery->cleanup();
  recovery.reset();
  native.cleanup();
  map_ros->cleanup();
  rclcpp::shutdown();
}
