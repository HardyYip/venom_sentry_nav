#pragma once
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace rmul2027_decision {
struct Goal {
  std::string key;
  double x=0, y=0, yaw=0;
};
struct Limits {
  double accept=2.0, navigation=180.0, feedback=2.0, cancel=2.0, retry=0.3;
  unsigned attempts=2;
  void validate() const {
    for(double v:{accept,navigation,feedback,cancel,retry})
      if(!std::isfinite(v)||v<=0) throw std::invalid_argument("Invalid action timeout");
    if(attempts<1||attempts>5) throw std::invalid_argument("Attempts must be 1..5");
  }
};
struct Effect { std::string kind, detail; unsigned token; std::optional<Goal> goal; };
class Executor {
 public:
  Limits limits;
  std::optional<Goal> desired, current;
  std::string state="idle", fault, completed;
  unsigned generation=0, attempts=0;
  bool accepted=false, cancelling=false, retry_cancel=false;
  double since=0, feedback_at=0, cancel_at=0, retry_at=0;
  std::vector<Effect> effects;
  std::function<void(const Goal&,double)> arrived;
  explicit Executor(Limits l={}):limits(l){limits.validate();}
  void emit(std::string kind,std::string detail="") {effects.push_back({kind,detail,generation,current});}
  void latch(const std::string &why) {if(fault.empty()){fault=why;emit("fault",why);}}
  void request(std::optional<Goal> next) {
    const std::string old=desired?desired->key:"", key=next?next->key:"";
    if(old!=key){attempts=0;completed.clear();retry_at=0;emit("intent",key);}
    desired=std::move(next);
  }
  bool same() const {return desired&&current&&desired->key==current->key;}
  void cancel(double now,bool retry=false) {
    if(!current||!accepted||cancelling)return;
    cancelling=true;retry_cancel=retry;cancel_at=now;state="cancelling";emit("cancel");
  }
  void failure(double now,const std::string &reason) {
    emit("attempt_failed",reason);
    if(!desired)return;
    if(attempts>=limits.attempts)latch(reason+"_exhausted");
    else {retry_at=now+limits.retry;state="retry_wait";}
  }
  void tick(double now) {
    if(current) {
      if(!accepted) {
        if(now-since>=limits.accept)latch("accept_timeout");
        return;
      }
      if(cancelling) {
        if(now-cancel_at>=limits.cancel)latch("cancel_timeout");
        return;
      }
      if(!fault.empty()||!same()){cancel(now);return;}
      if(now-since>=limits.navigation||now-feedback_at>=limits.feedback){
        emit("execution_timeout",now-since>=limits.navigation?"navigation":"feedback");
        cancel(now,true);
      }
      return;
    }
    if(!fault.empty()||!desired||completed==desired->key||now<retry_at)return;
    current=desired;accepted=false;cancelling=false;retry_cancel=false;
    ++generation;++attempts;since=now;feedback_at=now;state="sending";emit("send");
  }
  void response(unsigned token,bool ok,double now) {
    if(token!=generation||!current){emit("stale_response");return;}
    if(ok){accepted=true;state="active";feedback_at=now;emit("accepted");tick(now);}
    else {emit("rejected");current.reset();state="idle";if(fault.empty())failure(now,"rejected");}
  }
  void feedback(unsigned token,double now) {
    if(token==generation&&current&&accepted)feedback_at=now;
  }
  void cancel_response(unsigned token,bool ok) {
    if(token!=generation||!current)return;
    emit("cancel_ack",ok?"accepted":"rejected");
    if(!ok)latch("cancel_rejected");
  }
  void result(unsigned token,int code,double now) {
    if(token!=generation||!current){emit("stale_result");return;}
    if(code<1||code>3){latch("unknown_result");return;}
    Goal done=*current;bool was_cancel=cancelling, retry=retry_cancel, intended=same();
    emit("result",std::to_string(code));
    current.reset();accepted=false;cancelling=false;retry_cancel=false;state="idle";
    if(!fault.empty())return;
    if(was_cancel){if(retry&&intended)failure(now,"execution_timeout");return;}
    if(code==1&&intended){completed=done.key;if(arrived)arrived(done,now);}
    else if(intended)failure(now,code==2?"aborted":code==3?"unexpected_cancel":"unknown_result");
  }
  bool inhibited() const {
    return !fault.empty()||!desired||!current||!accepted||cancelling||!same();
  }
  bool reset() {
    if(current)return false;
    fault.clear();attempts=0;completed.clear();state="idle";return true;
  }
};

struct Input {
  int phase=0;
  double hp=400,ammo=100,heat=0;
  bool healthy=false;
};
struct PolicyConfig {
  double hp_low=100,hp_recover=200,ammo_low=10,ammo_recover=50,heat_high=80,heat_recover=40;
  double recover_hold=0.5,dwell=1.0;
  bool patrol=true;
  std::vector<Goal> points;
  Goal retreat;
  void validate() const {
    if(points.empty()||!std::isfinite(recover_hold)||recover_hold<0||!std::isfinite(dwell)||dwell<0||
       hp_recover<=hp_low||ammo_recover<=ammo_low||heat_recover>=heat_high)
      throw std::invalid_argument("Invalid policy/hysteresis/points");
    for(double v:{hp_low,hp_recover,ammo_low,ammo_recover,heat_high,heat_recover})
      if(!std::isfinite(v)||v<0)throw std::invalid_argument("Invalid threshold");
    for(const auto &g:points)if(!std::isfinite(g.x)||!std::isfinite(g.y)||!std::isfinite(g.yaw))
      throw std::invalid_argument("Invalid waypoint");
    if(!std::isfinite(retreat.x)||!std::isfinite(retreat.y)||!std::isfinite(retreat.yaw))
      throw std::invalid_argument("Invalid retreat");
  }
};
class Policy {
 public:
  PolicyConfig cfg;Input input;bool retreating=false;double healthy_since=-1,recover_since=-1,arrived_at=-1;
  size_t index=0;unsigned cycle=0;std::string mode="waiting",reason="startup";
  explicit Policy(PolicyConfig c):cfg(std::move(c)){cfg.validate();}
  void update(Input in,double now) {
    input=in;
    if(!in.healthy){healthy_since=-1;recover_since=-1;arrived_at=-1;return;}
    if(healthy_since<0)healthy_since=now;
    if(in.hp<=cfg.hp_low||in.ammo<=cfg.ammo_low||in.heat>=cfg.heat_high){retreating=true;recover_since=-1;}
    if(retreating&&in.hp>=cfg.hp_recover&&in.ammo>=cfg.ammo_recover&&in.heat<=cfg.heat_recover){
      if(recover_since<0)recover_since=now;
      if(now-recover_since>=cfg.recover_hold){retreating=false;recover_since=-1;arrived_at=-1;}
    }else recover_since=-1;
    if(!retreating&&cfg.patrol&&arrived_at>=0&&now-arrived_at>=cfg.dwell){
      arrived_at=-1;index=(index+1)%cfg.points.size();++cycle;
    }
  }
  bool healthy(double now) const {return input.healthy&&healthy_since>=0&&now-healthy_since>=cfg.recover_hold;}
  Goal task() const {
    Goal g=retreating?cfg.retreat:cfg.points[index];
    g.key=retreating?"retreat":"patrol:"+std::to_string(index)+":"+std::to_string(cycle);
    return g;
  }
  void arrival(const Goal &g,double now) {if(g.key.rfind("patrol:",0)==0)arrived_at=now;}
};
}
