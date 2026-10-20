#include "preprocess.h"
#include "ign_sim_pointcloud_tool/point_cloud_converter.hpp"
#include <algorithm>
#include <thread>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

void require(bool value, const std::string & message)
{
  if (!value) throw std::runtime_error(message);
}

sensor_msgs::msg::PointCloud2::SharedPtr cloud(const std::vector<float> & times)
{
  pcl::PointCloud<velodyne_ros::Point> input;
  for (size_t i = 0; i < times.size(); ++i) {
    velodyne_ros::Point point{};
    point.x = 2.0f;
    point.y = 0.2f * i;
    point.z = 0.0f;
    point.time = times[i];
    point.ring = 0;
    input.push_back(point);
  }
  auto result = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(input, *result);
  result->header.frame_id = "lidar";
  return result;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int checks = 0;
  try {
    Preprocess preprocess;
    preprocess.set(false, VELO16, 0.1, 1);
    preprocess.N_SCANS = 32;
    preprocess.time_unit = SEC;
    preprocess.point_time_is_valid = true;
    PointCloudXYZI::Ptr output(new PointCloudXYZI);
    preprocess.process(cloud({0, 0, 0}), output);
    require(output->size() == 3, "zero-time points were discarded");
    for (const auto & p : *output) require(p.curvature == 0, "snapshot was assigned fake scan offsets");
    ++checks;
    for (const auto & unit : std::vector<std::pair<int, float>>{{SEC, .025f}, {MS, 25.f}, {US, 25000.f}, {NS, 25000000.f}}) {
      preprocess.time_unit = unit.first;
      preprocess.process(cloud({0, unit.second, 2 * unit.second}), output);
      require(output->size() == 3, "provided points were discarded");
      require(std::abs((*output)[1].curvature - 25) < .001 && std::abs((*output)[2].curvature - 50) < .001,
        "point time conversion is not milliseconds");
      ++checks;
    }
    auto missing = cloud({0, 0, 0});
    missing->fields.erase(std::remove_if(missing->fields.begin(), missing->fields.end(),
      [](const auto & field) { return field.name == "time"; }), missing->fields.end());
    for (auto invalid : {missing, cloud({0, -1, 0}), cloud({0, std::numeric_limits<float>::quiet_NaN(), 0})}) {
      bool rejected = false;
      try { preprocess.process(invalid, output); } catch (const std::invalid_argument &) { rejected = true; }
      require(rejected, "invalid provided point time was accepted");
      ++checks;
    }
    preprocess.time_unit = SEC;
    preprocess.point_time_is_valid = false;
    preprocess.process(cloud({0, 0, 0}), output);
    require(output->size() == 2 && output->back().curvature > 0, "legacy automatic time behavior changed");
    ++checks;
    for (const std::string mode : {"legacy_rolling", "instantaneous"}) {
      auto options = rclcpp::NodeOptions().arguments({"--ros-args", "-r", "__ns:=/c022_point_time"})
        .parameter_overrides({rclcpp::Parameter("pcd_topic", "/c022_point_time/raw"), rclcpp::Parameter("point_time_mode", mode)});
      auto converter = std::make_shared<ign_sim_pointcloud_tool::PointCloudConverter>(options);
      auto observer = std::make_shared<rclcpp::Node>("point_time_observer", "/c022_point_time");
      sensor_msgs::msg::PointCloud2::SharedPtr received;
      auto subscription = observer->create_subscription<sensor_msgs::msg::PointCloud2>("velodyne_points", rclcpp::SensorDataQoS(),
        [&received](sensor_msgs::msg::PointCloud2::SharedPtr message) { received = message; });
      auto publisher = observer->create_publisher<sensor_msgs::msg::PointCloud2>("raw", 10);
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(converter); executor.add_node(observer);
      auto input = cloud({0, 0, 0});
      input->header.stamp.sec = 123;
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
      while (!received && std::chrono::steady_clock::now() < deadline) {
        publisher->publish(*input);
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      require(received != nullptr && received->header.stamp.sec == 123, "converter did not preserve source header");
      preprocess.point_time_is_valid = true;
      preprocess.process(received, output);
      require(output->size() == 3, "converter removed valid points");
      require(mode == "instantaneous" ? output->back().curvature == 0 : output->back().curvature > 0,
        "converter point_time_mode contract failed");
      ++checks;
    }
    std::cout << "{\"passed\":true,\"checks\":" << checks << "}" << std::endl;
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
