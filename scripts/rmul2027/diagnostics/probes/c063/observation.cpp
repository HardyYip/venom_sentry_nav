// Copyright 2026 HardyYip contributors
// SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <nav2_costmap_2d/observation_buffer.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <stdexcept>
#include <tf2_ros/transform_listener.h>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node =
      std::make_shared<nav2_util::LifecycleNode>("c063_observation_probe");
  tf2_ros::Buffer tf(node->get_clock());
  tf2_ros::TransformListener listener(tf, true);
  int checks = 0;
  for (auto frame : {"map", "odom"}) {
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = frame;
    t.child_frame_id = std::string(frame) + "_laser";
    t.transform.rotation.w = 1.;
    t.transform.translation.z = .03;
    if (!tf.setTransform(t, "c063 probe", true))
      throw std::runtime_error("TF setup failed");
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = t.child_frame_id;
    msg.header.stamp = node->now();
    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(5);
    sensor_msgs::PointCloud2Iterator<float> x(msg, "x"), y(msg, "y"),
        z(msg, "z");
    for (float h : {0.f, .029f, .031f, .12f, .22f}) {
      *x = 1.f;
      *y = 0.f;
      *z = h;
      ++x;
      ++y;
      ++z;
    }
    for (double min : {0., .06}) {
      nav2_costmap_2d::ObservationBuffer buffer(
          node, "probe", 0., 0., min, 2., 12., .15, 12., .15, tf, frame,
          t.child_frame_id, tf2::durationFromSec(0.));
      buffer.bufferCloud(msg);
      std::vector<nav2_costmap_2d::Observation> out;
      buffer.getObservations(out);
      if (out.size() != 1)
        throw std::runtime_error("Missing buffered cloud");
      auto cloud = out[0].cloud_;
      size_t n = cloud->width * cloud->height;
      size_t expected = min == 0. ? 5 : 3;
      if (n != expected)
        throw std::runtime_error("Incorrect actual buffer height selection");
      sensor_msgs::PointCloud2ConstIterator<float> oz(*cloud, "z");
      for (size_t i = 0; i < n; ++i, ++oz) {
        if (min > 0 && *oz < .06)
          throw std::runtime_error("Ground retained");
      }
      ++checks;
      std::cout << "{\"frame\":\"" << frame << "\",\"min_height\":" << min
                << ",\"retained\":" << n << ",\"passed\":true}" << std::endl;
    }
  }
  rclcpp::shutdown();
  std::cout << "{\"passed\":true,\"checks\":" << checks << "}" << std::endl;
}
