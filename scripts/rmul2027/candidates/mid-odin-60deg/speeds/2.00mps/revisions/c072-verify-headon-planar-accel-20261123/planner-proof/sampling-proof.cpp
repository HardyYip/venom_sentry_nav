// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <dwb_core/trajectory_generator.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <pluginlib/class_loader.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_util::LifecycleNode>("sampling_proof");
  const std::string name = "Controller";
  for (const auto & axis : {"x", "y"}) {
    const std::string suffix = std::string("_") + axis;
    node->declare_parameter(name + ".min_vel" + suffix, -2.0);
    node->declare_parameter(name + ".max_vel" + suffix, 2.0);
    node->declare_parameter(name + ".acc_lim" + suffix, 1.75);
    node->declare_parameter(name + ".decel_lim" + suffix, -1.75);
  }
  node->declare_parameter(name + ".max_vel_theta", 0.0);
  node->declare_parameter(name + ".min_speed_theta", 0.0);
  node->declare_parameter(name + ".min_speed_xy", 0.0);
  node->declare_parameter(name + ".max_speed_xy", 2.0);
  node->declare_parameter(name + ".vx_samples", 51);
  node->declare_parameter(name + ".vy_samples", 51);
  node->declare_parameter(name + ".vtheta_samples", 1);
  node->declare_parameter(name + ".max_accel_xy", 1.75);
  node->declare_parameter(name + ".discretize_by_time", true);
  node->declare_parameter(name + ".time_granularity", .05);
  node->declare_parameter(name + ".sim_time", 1.5);
  pluginlib::ClassLoader<dwb_core::TrajectoryGenerator> loader(
    "dwb_core", "dwb_core::TrajectoryGenerator");
  auto original = loader.createSharedInstance("dwb_plugins::StandardTrajectoryGenerator");
  auto candidate = loader.createSharedInstance("rmul_c072_planner::FineLowSpeedTrajectoryGenerator");
  original->initialize(node, name);
  candidate->initialize(node, name);
  nav_2d_msgs::msg::Twist2D motion;
  const auto nominal = original->getTwists(motion);
  const auto fine = candidate->getTwists(motion);
  if (fine.size() != nominal.size() + 128 ||
    !std::equal(nominal.begin(), nominal.end(), fine.begin() + 128))
  {
    throw std::runtime_error("Ordinary proposals changed");
  }
  const auto gentle_turn = [](const auto & v) {
    return v.x > 0.14 && v.x < 0.17 && v.y > 0.027 && v.y < 0.033;
  };
  if (!std::any_of(fine.begin(), fine.end(), gentle_turn) ||
    std::any_of(nominal.begin(), nominal.end(), gentle_turn))
  {
    throw std::runtime_error("A small steering correction was not added");
  }
  motion.x = .8;
  if (candidate->getTwists(motion) != original->getTwists(motion)) {
    throw std::runtime_error("Unexpected sampling outside low-speed conditions");
  }

  nav_2d_msgs::msg::Twist2D desired;
  desired.x = 1.4;
  desired.y = 1.4;
  const auto prediction = candidate->generateTrajectory({}, {}, desired);
  const auto & first_step = prediction.poses.at(1);
  if (std::abs(std::hypot(first_step.x, first_step.y) - 1.75 * .05 * .05) > 1.e-7) {
    throw std::runtime_error("Diagonal trajectory acceleration exceeded or missed its planar limit");
  }
  candidate->setSpeedLimit(.1, false);
  motion.x = 0.0;
  const auto limited = candidate->getTwists(motion);
  if (limited.empty() || std::any_of(limited.begin(), limited.end(), [](const auto & v) {
      return std::hypot(v.x, v.y) > .100001 || v.theta != 0.;
    }))
  {
    throw std::runtime_error("External speed limit violated");
  }
  std::cout << "PASS 128 additional near-stop samples and shallow angles; normal moving sample grid unchanged; absolute speed limit preserved; predicted diagonal acceleration capped\n";
  rclcpp::shutdown();
}
