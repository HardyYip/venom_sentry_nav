#include <chrono>
#include <iostream>
#include <thread>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rcl/time.h"
#include "rosgraph_msgs/msg/clock.hpp"
#include "tf2_ros/buffer.h"

int halted=0;
int starts=0;
class RunningProbe : public BT::StatefulActionNode {
public:
  RunningProbe(const std::string & name,const BT::NodeConfiguration & config)
  : BT::StatefulActionNode(name,config) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override {++starts;return BT::NodeStatus::RUNNING;}
  BT::NodeStatus onRunning() override {return BT::NodeStatus::RUNNING;}
  void onHalted() override {++halted;}
};
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  auto node=rclcpp::Node::make_shared("pose_fresh_contract");
  auto clock=node->get_clock();
  if(rcl_enable_ros_time_override(clock->get_clock_handle())!=RCL_RET_OK) return 2;
  auto set_now=[&](double seconds) {
    if(rcl_set_ros_time_override(clock->get_clock_handle(),int64_t(seconds*1e9))!=RCL_RET_OK)
      throw std::runtime_error("clock override failed");
  };
  set_now(100.);
  auto buffer=std::make_shared<tf2_ros::Buffer>(clock);
  BT::BehaviorTreeFactory factory; factory.registerFromPlugin(argv[1]);
  factory.registerNodeType<RunningProbe>("RunningProbe");
  auto board=BT::Blackboard::create();board->set("node",node);board->set("tf_buffer",buffer);
  auto tree=factory.createTreeFromText(
    R"(<root main_tree_to_execute="Main"><BehaviorTree ID="Main"><ReactiveSequence><PoseFresh parent="map" child="base" timeout="0.2" future_tolerance="0.05"/><RunningProbe/></ReactiveSequence></BehaviorTree></root>)",board);
  auto source=[&](double seconds) {
    geometry_msgs::msg::TransformStamped t; t.header.frame_id="map";t.child_frame_id="base";
    t.header.stamp=rclcpp::Time(int64_t(seconds*1e9));t.transform.rotation.w=1.;
    if(!buffer->setTransform(t,"fixture")) throw std::runtime_error("TF injection failed");
  };
  int checks=0;
  auto check=[&](bool condition,const char * name) {
    ++checks;std::cout<<name<<": "<<(condition?"PASS":"FAIL")<<std::endl;
    if(!condition) throw std::runtime_error(name);
  };
  check(tree.tickRoot()==BT::NodeStatus::FAILURE,"missing transform rejected");
  source(100.); check(tree.tickRoot()==BT::NodeStatus::RUNNING,"fresh transform permits navigation");
  set_now(100.3); check(tree.tickRoot()==BT::NodeStatus::FAILURE,"stale source aborts");
  check(halted==1,"running child halted on stale source");
  source(101.);check(tree.tickRoot()==BT::NodeStatus::FAILURE,"future transform rejected");
  buffer->clear();source(100.3);check(tree.tickRoot()==BT::NodeStatus::RUNNING,"valid source recovers after invalid future");
  for(int i=0;i<5;++i) {set_now(100.31+i*.01);source(100.31+i*.01);std::this_thread::sleep_for(std::chrono::milliseconds(55));check(tree.tickRoot()==BT::NodeStatus::RUNNING,"stationary robot with progressing stamp remains healthy");}
  std::this_thread::sleep_for(std::chrono::milliseconds(240));
  source(100.35);check(tree.tickRoot()==BT::NodeStatus::FAILURE,"replayed stamp with paused ROS clock rejected by wall timer");
  check(halted==2,"running child halted on frozen clock");
  set_now(100.36);source(100.36);check(tree.tickRoot()==BT::NodeStatus::RUNNING,"new source progresses after fault");
  tree.haltTree();
  if(rcl_disable_ros_time_override(clock->get_clock_handle())!=RCL_RET_OK) return 2;
  auto publisher=node->create_publisher<rosgraph_msgs::msg::Clock>("/clock",rclcpp::QoS(1).best_effort());
  buffer->clear();source(20.);
  auto simulation=factory.createTreeFromText(
    R"(<root main_tree_to_execute="Main"><BehaviorTree ID="Main"><ReactiveSequence><PoseFresh use_sim_time="true" parent="map" child="base" timeout="0.2" future_tolerance="0.05"/><RunningProbe/></ReactiveSequence></BehaviorTree></root>)",board);
  int initial_starts=starts;
  check(simulation.tickRoot()==BT::NodeStatus::RUNNING && starts==initial_starts,"no simulation clock means waiting without starting motion child");
  auto publish_clock=[&](double seconds) {
    rosgraph_msgs::msg::Clock msg; msg.clock=rclcpp::Time(int64_t(seconds*1e9));publisher->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    return simulation.tickRoot();
  };
  for(int i=0;i<100 && starts==initial_starts;++i) publish_clock(20.);
  check(starts==initial_starts+1,"dedicated simulation clock works when BT node clock is system time");
  check(clock->now().seconds()>1000000.,"test clocks really use different epochs");
  check(publish_clock(20.4)==BT::NodeStatus::FAILURE,"simulation source age rejects old pose");
  source(20.4);check(publish_clock(20.4)==BT::NodeStatus::RUNNING,"simulation fresh source recovers");
  std::this_thread::sleep_for(std::chrono::milliseconds(240));
  check(simulation.tickRoot()==BT::NodeStatus::FAILURE,"missing simulation clock stream halts navigation");
  simulation.haltTree();rclcpp::shutdown();std::cout<<"CHECKS="<<checks<<std::endl;
}
