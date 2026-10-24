#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <nlohmann/json.hpp>
#include <vector>
#include <rosgraph_msgs/msg/clock.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <ignition/transport/Node.hh>
#include <ignition/msgs/clock.pb.h>
#include <ignition/msgs/imu.pb.h>
#include <ignition/msgs/odometry.pb.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <mutex>
#include <string>
#include <thread>
struct Stream {double stamp=-1,received=-1,progress=-1;unsigned long count=0,backwards=0;};
std::atomic<bool> running{true};
void finish(int){running=false;}
double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
template<class T> double stamp(const T & m){return m.header.stamp.sec+m.header.stamp.nanosec*1.e-9;}
int main(int argc,char**argv){
 if(argc<2)return 2;
 rclcpp::init(argc,argv);
 std::signal(SIGINT,finish);std::signal(SIGTERM,finish);
 std::mutex mutex;
 std::map<std::string,Stream> streams;
 std::vector<nlohmann::json> failures;unsigned long stream_overflow=0,event_overflow=0;
 double native_real=-1,native_system=-1;
 auto receive=[&](const std::string &name,double value){
  if(name.size()>256||(streams.count(name)==0&&streams.size()>=128)){++stream_overflow;return;}
  auto &s=streams[name];const double t=steady();
  if(s.count==0||value>s.stamp)s.progress=t;
  if(s.count&&value<s.stamp-1.e-6)++s.backwards;
  s.stamp=value;s.received=t;++s.count;
 };
 ignition::transport::Node gz;
 gz.Subscribe("/world/default/clock",std::function<void(const ignition::msgs::Clock &)>([&](const auto&m){
  std::lock_guard<std::mutex> lock(mutex);
  receive("native_clock",m.sim().sec()+m.sim().nsec()*1.e-9);
  native_real=m.real().sec()+m.real().nsec()*1.e-9;native_system=m.system().sec()+m.system().nsec()*1.e-9;
 }));
 gz.Subscribe("/world/default/model/red_standard_robot1/link/front_mid360/sensor/front_mid360_imu/imu",std::function<void(const ignition::msgs::IMU &)>([&](const auto&m){
  std::lock_guard<std::mutex> lock(mutex);receive("native_imu",m.header().stamp().sec()+m.header().stamp().nsec()*1.e-9);
 }));
 gz.Subscribe("/red_standard_robot1/odometry",std::function<void(const ignition::msgs::Odometry &)>([&](const auto&m){
  std::lock_guard<std::mutex> lock(mutex);receive("native_odometry",m.header().stamp().sec()+m.header().stamp().nsec()*1.e-9);
 }));
 auto node=rclcpp::Node::make_shared("timing_chain_observer");
 std::vector<rclcpp::SubscriptionBase::SharedPtr> subs;
 for(const std::string topic:{"/rosout","/red_standard_robot1/rosout"}){
  subs.push_back(node->create_subscription<rcl_interfaces::msg::Log>(topic,rclcpp::QoS(100),
   [&](rcl_interfaces::msg::Log::ConstSharedPtr m){
    if(m->msg.find("PoseFresh FAILURE:")==std::string::npos)return;
    std::lock_guard<std::mutex> lock(mutex);
    if(failures.size()>=64){++event_overflow;return;}
    failures.push_back({{"source","rosout"},{"logger",m->name.substr(0,256)},{"message",m->msg.substr(0,2048)},
     {"log_stamp",m->stamp.sec+m->stamp.nanosec*1e-9},{"received",steady()}});
   }));
 }
 auto qos=rclcpp::QoS(1).best_effort();
 subs.push_back(node->create_subscription<rosgraph_msgs::msg::Clock>("/clock",qos,[&](rosgraph_msgs::msg::Clock::ConstSharedPtr m){std::lock_guard<std::mutex> lock(mutex);receive("ros_clock",m->clock.sec+m->clock.nanosec*1.e-9);}));
 const std::string ns="/red_standard_robot1/";
 for(const auto &name:{"aft_mapped_to_init","lidar_odometry","odometry","chassis_odometry_gt"}){
  std::string key=name;subs.push_back(node->create_subscription<nav_msgs::msg::Odometry>(ns+key,qos,[&,key](nav_msgs::msg::Odometry::ConstSharedPtr m){std::lock_guard<std::mutex> lock(mutex);receive(key,stamp(*m));}));
 }
 subs.push_back(node->create_subscription<sensor_msgs::msg::Imu>(ns+"livox/imu",qos,[&](sensor_msgs::msg::Imu::ConstSharedPtr m){std::lock_guard<std::mutex> lock(mutex);receive("imu",stamp(*m));}));
 for(const auto &name:{"livox/lidar","velodyne_points"}){
  std::string key=name;subs.push_back(node->create_subscription<sensor_msgs::msg::PointCloud2>(ns+key,qos,[&,key](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){std::lock_guard<std::mutex> lock(mutex);receive(key,stamp(*m));}));
 }
 subs.push_back(node->create_subscription<tf2_msgs::msg::TFMessage>(ns+"tf",rclcpp::QoS(50),[&](tf2_msgs::msg::TFMessage::ConstSharedPtr m){
  std::lock_guard<std::mutex> lock(mutex);
  for(const auto &t:m->transforms)receive("tf:"+t.header.frame_id+"->"+t.child_frame_id,stamp(t));
 }));
 rclcpp::executors::SingleThreadedExecutor executor;executor.add_node(node);
 std::thread ros([&](){executor.spin();});
 std::ofstream out(argv[1]);double previous=steady();
 while(running&&rclcpp::ok()){
  auto next=std::chrono::steady_clock::now()+std::chrono::milliseconds(50);
  const double t=steady(),wall=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  {
   std::lock_guard<std::mutex> lock(mutex);
   out<<std::setprecision(16)<<"{\"wall\":"<<wall<<",\"steady\":"<<t<<",\"loop_gap\":"<<t-previous<<",\"native_real\":"<<native_real<<",\"native_system\":"<<native_system<<",\"streams\":{";
   bool first=true;
   for(const auto &[name,s]:streams){
    if(!first)out<<",";first=false;
    out<<"\""<<name<<"\":{\"stamp\":"<<s.stamp<<",\"received\":"<<s.received<<",\"progress\":"<<s.progress<<",\"count\":"<<s.count<<",\"backwards\":"<<s.backwards<<"}";
   }
   out<<"},\"failures\":"<<nlohmann::json(failures).dump()<<",\"stream_overflow\":"<<stream_overflow<<",\"event_overflow\":"<<event_overflow<<"}\n";out.flush();failures.clear();
  }
  previous=t;std::this_thread::sleep_until(next);
 }
 executor.cancel();ros.join();gz.Unsubscribe("/world/default/clock");gz.Unsubscribe("/world/default/model/red_standard_robot1/link/front_mid360/sensor/front_mid360_imu/imu");gz.Unsubscribe("/red_standard_robot1/odometry");rclcpp::shutdown();return 0;
}
