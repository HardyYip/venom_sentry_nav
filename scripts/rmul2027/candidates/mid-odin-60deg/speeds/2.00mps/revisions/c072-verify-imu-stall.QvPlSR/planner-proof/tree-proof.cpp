#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>

using BT::NodeStatus;
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

void require(bool ok, const char * message)
{
  if (!ok) throw std::runtime_error(message);
}

struct Fixture
{
  bool recover;
  bool following{false};
  int plans{0};
  int waits{0};
  int starts{0};
  int halts{0};
};

class Planner : public BT::SyncActionNode
{
public:
  Planner(const std::string & name, const BT::NodeConfiguration & config, Fixture & data)
  : SyncActionNode(name, config), data_(data) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("goal"), BT::InputPort<std::string>("planner_id"),
      BT::OutputPort<std::string>("path")};
  }

private:
  NodeStatus tick() override
  {
    ++data_.plans;
    if (data_.plans == 1 || (data_.recover && data_.plans == 5)) {
      setOutput("path", std::to_string(data_.plans));
      return NodeStatus::SUCCESS;
    }
    setOutput("path", std::string{});
    return NodeStatus::FAILURE;
  }

  Fixture & data_;
};

class Controller : public BT::StatefulActionNode
{
public:
  Controller(const std::string & name, const BT::NodeConfiguration & config, Fixture & data)
  : StatefulActionNode(name, config), data_(data) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("path"), BT::InputPort<std::string>("controller_id")};
  }

private:
  NodeStatus onStart() override
  {
    ++data_.starts;
    const auto path = getInput<std::string>("path");
    require(static_cast<bool>(path), "followed a missing path");
    if (data_.plans == 1) {
      require(path.value() == "1", "initial route mismatch");
      data_.following = true;
      return NodeStatus::RUNNING;
    }
    require(data_.recover && data_.plans == 5 && path.value() == "5",
      "controller restarted before a new route was available");
    require(data_.halts == 1, "old controller handle was not halted");
    return NodeStatus::SUCCESS;
  }

  NodeStatus onRunning() override
  {
    require(data_.plans == 1, "old FollowPath ticked during a planning hold");
    return NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    ++data_.halts;
    data_.following = false;
  }

  Fixture & data_;
};

class Wait : public BT::StatefulActionNode
{
public:
  Wait(const std::string & name, const BT::NodeConfiguration & config, Fixture & data)
  : StatefulActionNode(name, config), data_(data) {}

  static BT::PortsList providedPorts() {return {BT::InputPort<int>("wait_duration")};}

private:
  NodeStatus onStart() override
  {
    const auto seconds = getInput<int>("wait_duration");
    require(seconds && seconds.value() == 1, "configured recovery wait changed");
    ++data_.waits;
    ends_ = Clock::now() + std::chrono::seconds(seconds.value());
    return NodeStatus::RUNNING;
  }

  NodeStatus onRunning() override
  {
    require(!data_.following, "old FollowPath remained active in the waiting interval");
    return Clock::now() >= ends_ ? NodeStatus::SUCCESS : NodeStatus::RUNNING;
  }

  void onHalted() override {}
  Fixture & data_;
  Clock::time_point ends_;
};

class Fresh : public BT::ConditionNode
{
public:
  using ConditionNode::ConditionNode;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("use_sim_time"), BT::InputPort<std::string>("parent"),
      BT::InputPort<std::string>("child"), BT::InputPort<std::string>("timeout"),
      BT::InputPort<std::string>("future_tolerance"), BT::InputPort<std::string>("initial_transform_wait")};
  }
private:
  NodeStatus tick() override {return NodeStatus::SUCCESS;}
};

class UnexpectedRecovery : public BT::SyncActionNode
{
public:
  using SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("service_name"), BT::InputPort<std::string>("backup_dist"),
      BT::InputPort<std::string>("backup_speed"), BT::InputPort<std::string>("time_allowance")};
  }
private:
  NodeStatus tick() override {throw std::runtime_error("unexpected clear or movement in planner recovery");}
};

int main(int argc, char ** argv)
{
  require(argc == 2, "expected production tree path");
  for (const bool recover : {true, false}) {
    Fixture data{recover};
    BT::BehaviorTreeFactory factory;
    for (const auto * plugin : {"nav2_pipeline_sequence_bt_node", "nav2_recovery_node_bt_node",
      "nav2_rate_controller_bt_node", "nav2_round_robin_node_bt_node"})
    {
      factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
    }
    factory.registerNodeType<Fresh>("PoseFresh");
    factory.registerBuilder<Planner>("ComputePathToPose",
      [&](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<Planner>(name, config, data);
      });
    factory.registerBuilder<Controller>("FollowPath",
      [&](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<Controller>(name, config, data);
      });
    factory.registerBuilder<Wait>("Wait",
      [&](const std::string & name, const BT::NodeConfiguration & config) {
        return std::make_unique<Wait>(name, config, data);
      });
    factory.registerNodeType<UnexpectedRecovery>("ClearEntireCostmap");
    factory.registerNodeType<UnexpectedRecovery>("BackUp");
    auto board = BT::Blackboard::create();
    board->set("goal", std::string("fixture"));
    auto tree = factory.createTreeFromFile(argv[1], board);
    NodeStatus status = NodeStatus::RUNNING;
    const auto deadline = Clock::now() + 8s;
    while (status == NodeStatus::RUNNING && Clock::now() < deadline) {
      status = tree.tickRoot();
      if (data.plans > 1) require(!data.following, "controller remained active after planner failure");
      std::this_thread::sleep_for(10ms);
    }
    require(status == (recover ? NodeStatus::SUCCESS : NodeStatus::FAILURE), "wrong terminal status");
    require(data.plans == 5 && data.halts == 1 && data.waits == 3, "unexpected retry or cancellation count");
    require(data.starts == (recover ? 2 : 1), "incorrect number of follow actions");
    std::cout << (recover ? "PASS transient no-path: halted, waited, followed fresh plan" :
      "PASS persistent no-path: halted, three bounded waits, failed safely") << '\n';
  }
}
