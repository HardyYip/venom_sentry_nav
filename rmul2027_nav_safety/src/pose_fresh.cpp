#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <mutex>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ros/buffer.h"

namespace pb_nav2_plugins
{
class PoseFresh : public BT::ConditionNode
{
public:
  PoseFresh(const std::string & name, const BT::NodeConfiguration & conf)
  : BT::ConditionNode(name, conf),
    node_(conf.blackboard->get<rclcpp::Node::SharedPtr>("node")),
    buffer_(conf.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer"))
  {
    sim_time_ = getInput<bool>("use_sim_time").value();
    started_ = std::chrono::steady_clock::now();
    if (sim_time_) {
      callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
      rclcpp::SubscriptionOptions options; options.callback_group = callback_group_;
      clock_sub_ = node_->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(1).best_effort(),
        [this](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(clock_mutex_);
          sim_now_ = rclcpp::Time(msg->clock).seconds();
          clock_received_ = std::chrono::steady_clock::now();
          clock_seen_ = true;
        }, options);
      executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
      clock_thread_ = std::thread([this]() {
        try {
          executor_.spin();
        } catch (const std::exception &) {
          std::lock_guard<std::mutex> lock(clock_mutex_);
          clock_executor_failed_ = true;
        }
      });
    }
  }

  ~PoseFresh() override
  {
    if (clock_thread_.joinable()) {
      executor_.cancel();
      clock_thread_.join();
    }
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("use_sim_time", false, "Read simulation clock independently of BT client executor"),
      BT::InputPort<std::string>("parent", std::string("map"), "Global frame"),
      BT::InputPort<std::string>("child", std::string("base_footprint"), "Tracked frame"),
      BT::InputPort<double>("timeout", 0.5, "Maximum source age and wall stall"),
      BT::InputPort<double>("future_tolerance", 0.1, "Maximum future timestamp")};
  }

  BT::NodeStatus tick() override
  {
    const double timeout = getInput<double>("timeout").value();
    const double future = getInput<double>("future_tolerance").value();
    if (!std::isfinite(timeout) || timeout <= 0.0 || !std::isfinite(future) || future < 0.0) {
      throw BT::RuntimeError("PoseFresh requires positive timeout and nonnegative future tolerance");
    }
    double current_time = 0.0;
    if (sim_time_) {
      bool clock_seen;
      bool clock_failed;
      std::chrono::steady_clock::time_point clock_received;
      {
        std::lock_guard<std::mutex> lock(clock_mutex_);
        clock_seen = clock_seen_;
        clock_failed = clock_executor_failed_;
        clock_received = clock_received_;
        current_time = sim_now_;
      }
      if (clock_failed) {
        RCLCPP_WARN(node_->get_logger(), "PoseFresh FAILURE: simulation_clock_executor_failed");
        return BT::NodeStatus::FAILURE;
      }
      if (!clock_seen) {
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - started_).count() <= timeout) {
          return BT::NodeStatus::RUNNING;
        }
        RCLCPP_WARN(node_->get_logger(), "PoseFresh FAILURE: simulation_clock_missing");
        return BT::NodeStatus::FAILURE;
      }
      if (std::chrono::duration<double>(std::chrono::steady_clock::now() - clock_received).count() > timeout) {
        RCLCPP_WARN(node_->get_logger(), "PoseFresh FAILURE: simulation_clock_receive_timeout");
        return BT::NodeStatus::FAILURE;
      }
    } else {
      current_time = node_->now().seconds();
    }
    std::string reason;
    double observed_age = 0.0;
    double observed_stamp = 0.0;
    try {
      const auto transform = buffer_->lookupTransform(
        getInput<std::string>("parent").value(), getInput<std::string>("child").value(),
        tf2::TimePointZero);
      const double stamp = rclcpp::Time(transform.header.stamp).seconds();
      const double age = current_time - stamp;
      observed_age = age; observed_stamp = stamp;
      const auto now = std::chrono::steady_clock::now();
      if (age >= -future && age <= timeout && (!seen_ || stamp > highwater_)) {
        seen_ = true;
        highwater_ = stamp;
        last_progress_ = now;
      }
      const auto & p = transform.transform.translation;
      const auto & q = transform.transform.rotation;
      const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(norm) || norm < 1e-12) {reason = "invalid_transform";}
      else if (age < -future) {reason = "source_future";}
      else if (age > timeout) {reason = "source_stale";}
      else if (std::chrono::duration<double>(now - last_progress_).count() > timeout) {
        reason = "stamp_stalled";
      }
    } catch (const tf2::TransformException &) {
      reason = "transform_missing";
    }
    if (reason != last_reason_) {
      if (!reason.empty()) {
        RCLCPP_WARN(node_->get_logger(), "PoseFresh FAILURE: %s; source=%.6f age=%.6f now=%.6f sim_active=%d; current goal must end",
          reason.c_str(), observed_stamp, observed_age, current_time,
          sim_time_);
      } else {
        RCLCPP_INFO(node_->get_logger(), "PoseFresh healthy; new goals may run");
      }
      last_reason_ = reason;
    }
    return reason.empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  bool sim_time_{false};
  bool clock_seen_{false};
  double sim_now_{0.0};
  std::chrono::steady_clock::time_point started_{}, clock_received_{};
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::mutex clock_mutex_;
  std::thread clock_thread_;
  bool clock_executor_failed_{false};
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  bool seen_{false};
  double highwater_{0.0};
  std::chrono::steady_clock::time_point last_progress_{};
  std::string last_reason_;
};
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb_nav2_plugins::PoseFresh>("PoseFresh");
}
