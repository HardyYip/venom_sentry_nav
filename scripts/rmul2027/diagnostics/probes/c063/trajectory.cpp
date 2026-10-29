// Copyright 2026 HardyYip contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <dwb_plugins/standard_traj_generator.hpp>
#include <iostream>
#include <stdexcept>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int checks = 0;
  for (bool timed : {false, true}) {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(
        {rclcpp::Parameter("FollowPath.sim_time", 1.5),
         rclcpp::Parameter("FollowPath.discretize_by_time", timed),
         rclcpp::Parameter("FollowPath.time_granularity", .05),
         rclcpp::Parameter("FollowPath.linear_granularity", .025),
         rclcpp::Parameter("FollowPath.angular_granularity", .025),
         rclcpp::Parameter("FollowPath.limit_vel_cmd_in_traj", true),
         rclcpp::Parameter("FollowPath.acc_lim_x", 1.),
         rclcpp::Parameter("FollowPath.decel_lim_x", -1.),
         rclcpp::Parameter("FollowPath.acc_lim_y", 1.),
         rclcpp::Parameter("FollowPath.decel_lim_y", -1.)});
    auto n = std::make_shared<nav2_util::LifecycleNode>(
        timed ? "timed_probe" : "distance_probe", "", opts);
    dwb_plugins::StandardTrajectoryGenerator gen;
    gen.initialize(n, "FollowPath");
    for (double command : {0., .05, .5}) {
      geometry_msgs::msg::Pose2D pose;
      nav_2d_msgs::msg::Twist2D vel, cmd;
      vel.x = .5;
      cmd.x = command;
      auto t = gen.generateTrajectory(pose, vel, cmd);
      double end = t.poses.back().x;
      if (command == 0. &&
          ((!timed && end != 0.) || (timed && !(end > .11 && end < .126))))
        throw std::runtime_error("Stopping-distance regression");
      if (command == .05 && ((timed && std::abs(end - .17625) > .0126) ||
                             (!timed && std::abs(end - .17625) < .05)))
        throw std::runtime_error("Low-speed braking model regression");
      if (command == .5 && std::abs(end - .75) > .00001)
        throw std::runtime_error("Cruise regression");
      if (timed && command == 0. && std::abs(t.velocity.x - .45) > .00001)
        throw std::runtime_error("Output does not match 20Hz control step");
      ++checks;
      std::cout << "{\"timed\":" << (timed ? "true" : "false")
                << ",\"target\":" << command << ",\"steps\":" << t.poses.size()
                << ",\"travel\":" << end << ",\"output_v\":" << t.velocity.x
                << ",\"passed\":true}" << std::endl;
    }
  }
  rclcpp::shutdown();
  std::cout << "{\"checks\":" << checks << ",\"passed\":true}" << std::endl;
}
