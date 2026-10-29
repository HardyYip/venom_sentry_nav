#include "rmul2027_decision/tree.hpp"
#include "rmul2027_decision/msg/match_state.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/contrib/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <action_msgs/msg/goal_status_array.hpp>
#include <csignal>
#include <set>
#include <thread>
#include <nav_msgs/msg/odometry.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>

using namespace rmul2027_decision;
using Json=nlohmann::json;
using Action=nav2_msgs::action::NavigateToPose;
using Handle=rclcpp_action::ClientGoalHandle<Action>;
using Bool=example_interfaces::msg::Bool;
static double steady(){
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static std::string uuid(const rclcpp_action::GoalUUID &id){
  std::ostringstream s;for(auto v:id)s<<std::hex<<std::setfill('0')<<std::setw(2)<<int(v);return s.str();
}
class Decision:public rclcpp::Node {
 public:
  Decision():Node("rmul2027_decision"){
    const auto number=[&](const char *n,double d){double v=declare_parameter<double>(n,d);
      if(!std::isfinite(v)||v<=0){throw std::invalid_argument(n);}
      return v;};
    ref_timeout_=number("referee_timeout",0.4);ref_source_=number("referee_source_timeout",0.4);
    loc_timeout_=number("localization_timeout",0.5);input_timeout_=number("input_timeout",0.4);
    recovery_timeout_=declare_parameter<double>("recovery_permit_timeout",0.0);
    if(!std::isfinite(recovery_timeout_)||recovery_timeout_<0)throw std::invalid_argument("recovery_permit_timeout");
    if(recovery_timeout_>0)recovery_=create_subscription<Bool>("recovery/permit",1,
      [this](Bool::ConstSharedPtr m){recovery_permit_=m->data;recovery_at_=steady();});
    PolicyConfig c;
    c.hp_low=declare_parameter<double>("hp_low",100);c.hp_recover=declare_parameter<double>("hp_recover",200);
    c.ammo_low=declare_parameter<double>("ammo_low",10);c.ammo_recover=declare_parameter<double>("ammo_recover",50);
    c.heat_high=declare_parameter<double>("heat_high",80);c.heat_recover=declare_parameter<double>("heat_recover",40);
    c.recover_hold=number("recovery_hold",0.5);c.dwell=number("dwell_seconds",1);
    c.patrol=declare_parameter<bool>("patrol",true);
    auto xs=declare_parameter<std::vector<double>>("waypoint_x",{7.5,6.5});
    auto ys=declare_parameter<std::vector<double>>("waypoint_y",{-0.2,-0.2});
    auto yaws=declare_parameter<std::vector<double>>("waypoint_yaw",{0,0});
    if(xs.size()!=ys.size()||xs.size()!=yaws.size())throw std::invalid_argument("Waypoint lengths");
    for(size_t i=0;i<xs.size();++i)c.points.push_back({"",xs[i],ys[i],yaws[i]});
    auto rt=declare_parameter<std::vector<double>>("retreat",{8.5,-0.2,0});
    if(rt.size()!=3)throw std::invalid_argument("Retreat length");
    c.retreat={"retreat",rt[0],rt[1],rt[2]};policy_=std::make_unique<Policy>(c);
    Limits l;l.accept=number("accept_timeout",2);l.navigation=number("navigation_timeout",180);
    l.feedback=number("feedback_timeout",2);l.cancel=number("cancel_timeout",2);l.retry=number("retry_delay",0.3);
    int attempts=declare_parameter<int>("max_attempts",2);
    if(attempts<1||attempts>5)throw std::invalid_argument("max_attempts");
    l.attempts=unsigned(attempts);executor_=std::make_unique<Executor>(l);
    executor_->arrived=[this](const Goal&g,double t){policy_->arrival(g,t);};
    auto label=declare_parameter<std::string>("strategy_label","TEST ONLY: not RMUL2027 rules");
    const auto file=declare_parameter<std::string>("event_file","");
    if(!file.empty()){file_.open(file,std::ios::app);if(!file_)throw std::runtime_error("Cannot open event_file");}
    const auto ns=std::string(get_namespace());
    std::string lock="/tmp/rmul2027-decision"+ns+".lock";
    for(size_t i=5;i<lock.size();++i)if(lock[i]=='/')lock[i]='_';
    lock_=open(lock.c_str(),O_CREAT|O_RDWR,0600);
    if(lock_<0||flock(lock_,LOCK_EX|LOCK_NB)!=0)throw std::runtime_error("Decision process already owns namespace");
    events_=create_publisher<std_msgs::msg::String>("decision/events",100);
    status_=create_publisher<std_msgs::msg::String>("decision/status",10);
    stop_=create_publisher<Bool>("decision_stop",1);
    client_=rclcpp_action::create_client<Action>(this,"navigate_to_pose");
    buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    listener_=std::make_unique<tf2_ros::TransformListener>(*buffer_,this,false);
    match_=create_subscription<msg::MatchState>(declare_parameter<std::string>("match_topic","decision/match"),10,
      [this](msg::MatchState::ConstSharedPtr m){
        const double source=rclcpp::Time(m->stamp).seconds();
        const bool values=m->phase<=2&&m->valid&&!m->source.empty()&&
          std::isfinite(m->hp)&&m->hp>=0&&std::isfinite(m->ammo)&&m->ammo>=0&&std::isfinite(m->heat)&&m->heat>=0;
        if(!values){ref_valid_=false;return;}
        if(have_ref_&&(m->sequence<=sequence_||source<=ref_stamp_))return;
        const double age=now().seconds()-source;
        if(age>ref_source_||age<-.1){ref_valid_=false;return;}
        have_ref_=true;ref_valid_=true;sequence_=m->sequence;ref_stamp_=source;ref_at_=steady();
        input_.phase=m->phase;input_.hp=m->hp;input_.ammo=m->ammo;input_.heat=m->heat;
        if(m->phase==2)finished_=true;
        emit("match",{{"sequence",sequence_},{"source_stamp",source},{"phase",m->phase},
          {"hp",m->hp},{"ammo",m->ammo},{"heat",m->heat},{"source",m->source}});
      });
    estop_=create_subscription<Bool>("decision/estop",1,[this](Bool::ConstSharedPtr m){estop_value_=m->data;estop_at_=steady();});
    trust_=create_subscription<Bool>("decision/localization_trusted",1,[this](Bool::ConstSharedPtr m){trusted_=m->data;trust_at_=steady();});
    odom_=create_subscription<nav_msgs::msg::Odometry>("odometry",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr m){
        odom_at_=steady();double stamp=rclcpp::Time(m->header.stamp).seconds();
        const auto &p=m->pose.pose.position;const auto &q=m->pose.pose.orientation;
        double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
        odom_valid_=std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(norm)&&std::abs(norm-1)<.01;
        if(stamp>odom_stamp_){odom_stamp_=stamp;odom_progress_=odom_at_;}
      });
    action_status_=create_subscription<action_msgs::msg::GoalStatusArray>(
      "navigate_to_pose/_action/status",rclcpp::QoS(1).transient_local(),
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr m){foreign_status_=m->status_list;});
    reset_=create_service<std_srvs::srv::Trigger>("decision/reset_fault",[this](
      const std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr res){
        res->success=input_.phase==0&&!finished_&&executor_->reset();
        res->message=res->success?"Reset in waiting phase":"Requires waiting phase, no owned/pending goal, no finished latch";
      });
    const auto tree=declare_parameter<std::string>("tree_file",
      ament_index_cpp::get_package_share_directory("rmul2027_decision")+"/behavior_trees/rmul_2027.xml");
    tree_=make_tree(factory_,*policy_,*executor_,tick_now_,tree);
    emit("startup",{{"strategy_label",label},{"bt_version","4.6.2"},{"namespace",ns}});
    timer_=create_wall_timer(std::chrono::milliseconds(20),[this]{tick();});
  }
  void begin_shutdown(){shutting_down_=true;emit("shutdown_requested");tick();}
  bool settled()const{return !executor_->current;}
  double shutdown_budget()const{return executor_->limits.cancel+.5;}
  void shutdown_record(){emit(settled()?"shutdown_terminal_confirmed":"shutdown_unconfirmed_inhibited");}
  ~Decision()override{if(lock_>=0){flock(lock_,LOCK_UN);close(lock_);}}
 private:
  void emit(const std::string &kind,Json data=Json::object()){
    data["kind"]=kind;data["steady"]=steady();data["ros"]=now().seconds();
    std_msgs::msg::String m;m.data=data.dump();events_->publish(m);
    if(file_){file_<<m.data<<"\n";file_.flush();}
  }
  std::string health(double t){
    if(shutting_down_)return "shutdown";
    if(t-started_<1.0)return "startup_discovery";
    if(!(executor_->current&&!executor_->accepted)){
      for(const auto &g:foreign_status_)if(g.status>=1&&g.status<=3&&owned_ids_.count(uuid(g.goal_info.goal_id.uuid))==0){
        executor_->latch("foreign_active_goal");return "foreign_active_goal";
      }
    }
    if(finished_)return "match_finished";
    if(!ref_valid_||t-ref_at_>ref_timeout_)return "referee_stale_or_invalid";
    double age=now().seconds()-ref_stamp_;
    if(age>ref_source_||age<-.1)return "referee_source_age";
    if(estop_value_||t-estop_at_>input_timeout_)return "estop_or_missing_heartbeat";
    if(recovery_timeout_>0&&(!recovery_permit_||t-recovery_at_>recovery_timeout_))return "recovery_hold_or_missing_heartbeat";
    if(!trusted_||t-trust_at_>input_timeout_)return "localization_untrusted_or_missing_heartbeat";
    if(!odom_valid_||t-odom_at_>loc_timeout_||t-odom_progress_>loc_timeout_)return "odometry_invalid_or_stalled";
    age=now().seconds()-odom_stamp_;
    if(age>loc_timeout_||age<-.1)return "odometry_source_age";
    try{
      auto tf=buffer_->lookupTransform("map","base_footprint",tf2::TimePointZero);
      double ts=rclcpp::Time(tf.header.stamp).seconds();
      auto &p=tf.transform.translation;auto &q=tf.transform.rotation;
      double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
      if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(norm)||std::abs(norm-1)>.01)return "tf_invalid";
      if(ts>tf_stamp_){tf_stamp_=ts;tf_progress_=t;}
      age=now().seconds()-ts;
      if(age>loc_timeout_||age<-.1||t-tf_progress_>loc_timeout_)return "tf_stale_or_stalled";
    }catch(const tf2::TransformException&){return "tf_missing";}
    return "";
  }
  void send(const Effect &e){
    if(!client_->action_server_is_ready()){
      emit("server_unavailable",{{"token",e.token}});
      executor_->response(e.token,false,tick_now_);return;
    }
    Action::Goal goal;goal.pose.header.frame_id="map";goal.pose.header.stamp=now();
    goal.pose.pose.position.x=e.goal->x;goal.pose.pose.position.y=e.goal->y;
    goal.pose.pose.orientation.z=std::sin(e.goal->yaw/2);goal.pose.pose.orientation.w=std::cos(e.goal->yaw/2);
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback=[this,token=e.token](Handle::SharedPtr h){
      if(h){handles_[token]=h;owned_ids_.insert(uuid(h->get_goal_id()));}
      emit("goal_response",{{"token",token},{"accepted",bool(h)},{"uuid",h?uuid(h->get_goal_id()):""}});
      executor_->response(token,bool(h),steady());
    };
    options.feedback_callback=[this,token=e.token](Handle::SharedPtr h,const std::shared_ptr<const Action::Feedback> f){
      if(!std::isfinite(f->distance_remaining)){executor_->latch("invalid_feedback");return;}
      executor_->feedback(token,steady());
      if(steady()-last_feedback_event_>.2){
        last_feedback_event_=steady();emit("feedback",{{"token",token},{"uuid",uuid(h->get_goal_id())},
          {"distance_remaining",f->distance_remaining}});
      }
    };
    options.result_callback=[this,token=e.token](const Handle::WrappedResult &r){
      int code=r.code==rclcpp_action::ResultCode::SUCCEEDED?1:r.code==rclcpp_action::ResultCode::ABORTED?2:
        r.code==rclcpp_action::ResultCode::CANCELED?3:0;
      emit("action_result",{{"token",token},{"uuid",uuid(r.goal_id)},{"code",code}});
      executor_->result(token,code,steady());handles_.erase(token);
    };
    client_->async_send_goal(goal,options);
  }
  void cancel(const Effect &e){
    auto h=handles_.find(e.token);
    if(h==handles_.end()){executor_->latch("missing_owned_handle");return;}
    const auto id=h->second->get_goal_id();
    client_->async_cancel_goal(h->second,[this,token=e.token,id](auto result){
      bool found=false;for(auto &g:result->goals_canceling)if(g.goal_id.uuid==id)found=true;
      bool ok=result->return_code==0&&found;
      emit("cancel_response",{{"token",token},{"uuid",uuid(id)},{"return_code",result->return_code},{"matched",found}});
      executor_->cancel_response(token,ok);
    });
  }
  void tick(){
    tick_now_=steady();auto reason=health(tick_now_);input_.healthy=reason.empty();
    policy_->update(input_,tick_now_);policy_->reason=reason;
    tree_.tickExactlyOnce();executor_->tick(tick_now_);
    Bool stop;stop.data=executor_->inhibited();stop_->publish(stop);
    for(int n=0;n<10&&!executor_->effects.empty();++n){
      auto effects=std::move(executor_->effects);executor_->effects.clear();
      for(const auto &e:effects){
        Json j={{"token",e.token},{"detail",e.detail}};
        if(e.goal)j["goal"]={{"key",e.goal->key},{"x",e.goal->x},{"y",e.goal->y},{"yaw",e.goal->yaw}};
        emit(e.kind,j);
        try{if(e.kind=="send")send(e);else if(e.kind=="cancel")cancel(e);}
        catch(const std::exception &ex){executor_->latch(std::string("transport_exception:")+ex.what());}
      }
    }
    stop.data=executor_->inhibited();stop_->publish(stop);
    if(tick_now_-last_status_>.1){
      last_status_=tick_now_;
      Json j={{"steady",tick_now_},{"ros",now().seconds()},{"mode",policy_->mode},{"reason",reason},
        {"state",executor_->state},{"recovery_permit_timeout",recovery_timeout_},{"fault",executor_->fault},{"inhibited",stop.data},
        {"goal",executor_->current?executor_->current->key:""},{"token",executor_->generation}};
      std_msgs::msg::String m;m.data=j.dump();status_->publish(m);
      if(reason!=last_reason_){last_reason_=reason;emit("health",{{"reason",reason}});}
    }
  }
  double started_=steady();bool shutting_down_=false;
  std::set<std::string> owned_ids_;
  std::vector<action_msgs::msg::GoalStatus> foreign_status_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr action_status_;
  int lock_=-1;double tick_now_=0,last_status_=0,last_feedback_event_=0;
  double ref_timeout_=0,ref_source_=0,loc_timeout_=0,input_timeout_=0;
  double ref_at_=-1e20,ref_stamp_=0,estop_at_=-1e20,trust_at_=-1e20;
  double odom_at_=-1e20,odom_stamp_=0,odom_progress_=-1e20,tf_stamp_=0,tf_progress_=-1e20;
  uint64_t sequence_=0;bool have_ref_=false,ref_valid_=false,estop_value_=true,trusted_=false,odom_valid_=false,finished_=false;
  std::string last_reason_;std::ofstream file_;Input input_;
  std::unique_ptr<Policy> policy_;std::unique_ptr<Executor> executor_;
  BT::BehaviorTreeFactory factory_;BT::Tree tree_;
  rclcpp_action::Client<Action>::SharedPtr client_;std::map<unsigned,Handle::SharedPtr> handles_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_,status_;
  rclcpp::Publisher<Bool>::SharedPtr stop_;rclcpp::Subscription<msg::MatchState>::SharedPtr match_;
  rclcpp::Subscription<Bool>::SharedPtr estop_,trust_,recovery_;
  double recovery_timeout_=0,recovery_at_=-1e20;bool recovery_permit_=false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;rclcpp::TimerBase::SharedPtr timer_;
};
static volatile std::sig_atomic_t stop_requested=0;
static void request_stop(int){stop_requested=1;}
int main(int argc,char **argv){
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,request_stop);std::signal(SIGTERM,request_stop);
  try{
    auto node=std::make_shared<Decision>();rclcpp::executors::SingleThreadedExecutor runner;runner.add_node(node);
    while(rclcpp::ok()&&!stop_requested){runner.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(3));}
    if(rclcpp::ok()){
      node->begin_shutdown();const double end=steady()+node->shutdown_budget();
      while(rclcpp::ok()&&!node->settled()&&steady()<end){runner.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(3));}
      node->shutdown_record();
    }
    runner.remove_node(node);
  }catch(const std::exception &e){std::cerr<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
