#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <nlohmann/json.hpp>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
using J=nlohmann::json;using namespace std::chrono_literals;
int main(int argc,char**argv){
 rclcpp::init(argc,argv);auto pubnode=std::make_shared<rclcpp::Node>("clock_probe_publisher");auto pub=pubnode->create_publisher<rosgraph_msgs::msg::Clock>("/clock",rclcpp::QoS(10));std::atomic<bool>stop{false},pause{false},hold{false};std::atomic<double>latest{100.};
 std::thread producer([&]{while(!stop){if(!pause){double t=latest.load()+(hold?0.:.001);rosgraph_msgs::msg::Clock m;m.clock=rclcpp::Time(static_cast<int64_t>(t*1e9));pub->publish(m);latest=t;}std::this_thread::sleep_for(1ms);}});
 auto node=std::make_shared<rclcpp::Node>("posefresh_actual_clock_probe",rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time",true)}));auto buffer=std::make_shared<tf2_ros::Buffer>(node->get_clock());BT::BehaviorTreeFactory f;f.registerFromPlugin(argv[1]);auto bb=BT::Blackboard::create();bb->set("node",node);bb->set("tf_buffer",buffer);auto tree=f.createTreeFromText("<root main_tree_to_execute='T'><BehaviorTree ID='T'><PoseFresh use_sim_time='true' parent='map' child='base_footprint' timeout='0.5' future_tolerance='0.1'/></BehaviorTree></root>",bb);
 auto transform=[&]{geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";t.child_frame_id="base_footprint";t.header.stamp=rclcpp::Time(static_cast<int64_t>((latest.load()-.01)*1e9));t.transform.rotation.w=1.;buffer->setTransform(t,"clock_probe");};
 J out=J::array();
 for(int i=0;i<6;++i){std::this_thread::sleep_for(i==0?2s:500ms);transform();auto state=tree.tickRoot();out.push_back({{"case","fresh_after_idle"},{"iteration",i},{"status",BT::toStr(state)},{"publisher_clock",latest.load()},{"node_clock",node->now().seconds()}});}
 std::this_thread::sleep_for(100ms);pause=true;std::this_thread::sleep_for(750ms);transform();auto stalled=tree.tickRoot();out.push_back({{"case","clock_publisher_stopped_750ms"},{"status",BT::toStr(stalled)},{"publisher_clock",latest.load()}});
 pause=false;std::this_thread::sleep_for(150ms);transform();out.push_back({{"case","clock_resumed_fresh_pose"},{"status",BT::toStr(tree.tickRoot())}});
 auto special=[&](const std::string&name,double offset){buffer->clear();geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";t.child_frame_id="base_footprint";t.header.stamp=rclcpp::Time(static_cast<int64_t>((latest.load()+offset)*1e9));t.transform.rotation.w=1.;buffer->setTransform(t,"clock_probe");out.push_back({{"case",name},{"status",BT::toStr(tree.tickRoot())}});};
 special("original_source_age_1s_rejected",-1.);special("original_future_0p5s_rejected",.5);buffer->clear();out.push_back({{"case","original_missing_transform_rejected"},{"status",BT::toStr(tree.tickRoot())}});
 hold=true;std::this_thread::sleep_for(20ms);transform();out.push_back({{"case","fresh_before_stamp_stall"},{"status",BT::toStr(tree.tickRoot())}});std::this_thread::sleep_for(750ms);out.push_back({{"case","original_stamp_stalled_750ms_rejected"},{"status",BT::toStr(tree.tickRoot())}});
 stop=true;producer.join();tree.haltTree();std::cout<<"CLOCK_PROBE_JSON "<<out.dump()<<std::endl;rclcpp::shutdown();
}
