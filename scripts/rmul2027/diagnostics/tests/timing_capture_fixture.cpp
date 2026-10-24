#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/clock.pb.h>
#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/odometry.pb.h>
#include <chrono>
#include <thread>
#include <string>
#include <cstdlib>
#include <iostream>
int main(int argc,char**argv){
 const auto *domain=std::getenv("ROS_DOMAIN_ID"),*partition=std::getenv("IGN_PARTITION");
 if(!domain||std::string(domain)!="91"||!partition||std::string(partition)!="codex_c054_fixture"||!std::getenv("RMUL_STORAGE_RUN_ID")){
  std::cerr<<"Fixture requires guarded private ROS_DOMAIN_ID91 and IGN_PARTITIONcodex_c054_fixture\n";return 2;
 }
 rclcpp::init(argc,argv);std::string mode=argc>1?argv[1]:"healthy";
 auto node=rclcpp::Node::make_shared("timing_fixture");
 auto ros=node->create_publisher<rosgraph_msgs::msg::Clock>("/clock",rclcpp::QoS(1).best_effort());
 auto tf=node->create_publisher<tf2_msgs::msg::TFMessage>("/red_standard_robot1/tf",50);
 ignition::transport::Node gz;auto native=gz.Advertise<ignition::msgs::Clock>("/world/default/clock");
 auto native_imu=gz.Advertise<ignition::msgs::IMU>("/world/default/model/red_standard_robot1/link/front_mid360/sensor/front_mid360_imu/imu");auto native_odo=gz.Advertise<ignition::msgs::Odometry>("/red_standard_robot1/odometry");
 auto begin=std::chrono::steady_clock::now();bool logged=false;
 auto stamp=[](double t){return static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(static_cast<int64_t>(t*1e9)));};
 while(rclcpp::ok()){
  double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  if(elapsed>=(mode=="early_then_failure"?10.5:7.))break;
  double sim=20.+.1*elapsed,frozen=20.+.1*std::min(elapsed,3.5);
  ignition::msgs::Clock m;m.mutable_sim()->set_sec(static_cast<int64_t>(sim));
  m.mutable_sim()->set_nsec(static_cast<int64_t>((sim-static_cast<int64_t>(sim))*1e9));
  m.mutable_real()->set_sec(static_cast<int64_t>(elapsed));native.Publish(m);
  ignition::msgs::IMU ni;*ni.mutable_header()->mutable_stamp()=m.sim();native_imu.Publish(ni);
  ignition::msgs::Odometry no;*no.mutable_header()->mutable_stamp()=m.sim();native_odo.Publish(no);
  rosgraph_msgs::msg::Clock c;c.clock=stamp(mode=="bridge_freeze"?frozen:sim);ros->publish(c);
  double source=(mode=="tf_freeze"||mode=="bridge_freeze")?frozen:sim;
  if(mode=="early_then_failure"&&elapsed>=2.&&elapsed<2.8)source=20.2;
  tf2_msgs::msg::TFMessage transforms;
  for(auto frames:{std::pair<std::string,std::string>{"map","odom"},{"odom","base_footprint"}}){
   geometry_msgs::msg::TransformStamped t;t.header.frame_id=frames.first;t.child_frame_id=frames.second;
   t.header.stamp=stamp(source);t.transform.rotation.w=1.;transforms.transforms.push_back(t);
  }
  tf->publish(transforms);
  if(((mode=="rosout"&&elapsed>=3.5)||(mode=="early_then_failure"&&elapsed>=7.))&&!logged){
   RCLCPP_WARN(node->get_logger(),"PoseFresh FAILURE: injected_contract_test; fixture only, no navigation");logged=true;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
 }
 rclcpp::shutdown();return 0;
}
