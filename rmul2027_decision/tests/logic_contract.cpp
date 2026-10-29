#include "rmul2027_decision/tree.hpp"
#include <iostream>
using namespace rmul2027_decision;
void require(bool v,const char *why){if(!v)throw std::runtime_error(why);}
Goal a{"a",1,2,0},b{"b",2,2,0};
int main(int argc,char **argv){
 try{
  require(argc==2,"tree path");
  Executor e;e.request(a);e.tick(0);auto token=e.generation;
  e.response(token,true,.1);
  for(int i=0;i<20;++i){e.request(a);e.feedback(token,.2+i*.01);e.tick(.2+i*.01);}
  require(e.generation==1,"duplicate goal");
  e.request(b);e.tick(.5);require(e.cancelling,"switch cancel");
  e.cancel_response(token,true);e.tick(.6);require(e.generation==1,"ack is not terminal");
  e.result(token,3,.7);e.tick(.8);require(e.generation==2,"send after terminal");
  e.response(2,true,.9);e.request(a);e.tick(1);e.cancel_response(2,false);e.tick(10);
  require(e.fault=="cancel_rejected"&&e.generation==2&&e.inhibited()&&!e.reset(),"cancel rejected latch");
  e.result(2,3,11);require(e.reset(),"reset after terminal");
  Executor late;late.request(a);late.tick(0);late.tick(3);
  require(late.fault=="accept_timeout"&&late.current,"ambiguous send retained");
  late.response(1,true,4);require(late.cancelling&&late.inhibited(),"late accepted cancel");
  late.result(1,3,4.1);late.tick(5);require(late.generation==1,"late fault blocks new");
  Executor cancel;cancel.request(a);cancel.tick(0);cancel.response(1,true,.1);
  cancel.request(b);cancel.tick(.2);cancel.cancel_response(1,true);cancel.tick(3);
  require(cancel.fault=="cancel_timeout"&&cancel.current&&cancel.generation==1,"cancel terminal timeout");
  Executor reject;reject.request(a);reject.tick(0);reject.response(1,false,.1);reject.tick(.5);
  reject.response(2,false,.6);reject.tick(2);
  require(reject.fault=="rejected_exhausted"&&reject.generation==2,"bounded reject");
  Executor timeout;timeout.request(a);timeout.tick(0);timeout.response(1,true,.1);timeout.tick(3);
  require(timeout.cancelling&&timeout.retry_cancel,"feedback timeout cancel");
  timeout.result(1,3,3.1);timeout.tick(3.5);require(timeout.generation==2,"retry after terminal");
  timeout.response(2,true,3.6);timeout.result(1,1,3.7);
  require(timeout.current&&timeout.generation==2,"stale result ignored");
  timeout.result(2,1,3.8);timeout.tick(5);require(timeout.generation==2,"completed dedup");
  Executor unknown;unknown.request(a);unknown.tick(0);unknown.response(1,true,.1);unknown.result(1,0,.2);
  require(unknown.current&&unknown.fault=="unknown_result"&&unknown.inhibited(),"unknown result retains ownership");
  Executor safety;safety.request(a);safety.tick(0);safety.response(1,true,.1);
  safety.request(std::nullopt);safety.tick(.2);require(safety.cancelling&&safety.inhibited(),"safety priority");
  PolicyConfig c;c.points={a,b};c.retreat=b;Policy p(c);Executor x;
  double now=0;BT::BehaviorTreeFactory factory;auto tree=make_tree(factory,p,x,now,argv[1]);
  auto tick=[&](Input in,double t){now=t;p.update(in,t);tree.tickExactlyOnce();x.tick(t);};
  Input in;in.healthy=true;tick(in,0);tick(in,1);require(!x.current,"waiting no goal");
  in.phase=1;tick(in,1.1);require(x.current&&x.current->key=="patrol:0:0","start once");
  x.response(1,true,1.2);in.hp=90;tick(in,1.3);require(x.cancelling,"low hp preempt");
  x.result(1,3,1.4);tick(in,1.5);require(x.current->key=="retreat","retreat");
  x.response(2,true,1.6);x.result(2,1,1.7);in.hp=250;tick(in,2);tick(in,2.6);
  require(x.current&&x.current->key=="patrol:0:0","recover resume");
  x.response(3,true,2.7);x.arrived=[&](const Goal &g,double t){p.arrival(g,t);};x.result(3,1,2.8);
  tick(in,3);require(!x.current,"dwell after success");
  tick(in,4);require(x.current&&x.current->key=="patrol:1:1","patrol progression");
  in.healthy=false;tick(in,4.1);require(x.inhibited(),"invalid stop");
  in.phase=2;in.healthy=true;tick(in,5);require(!x.desired,"ended stop");
  PolicyConfig bad=c;bad.hp_recover=bad.hp_low;bool thrown=false;
  try{Policy fail(bad);}catch(const std::invalid_argument&){thrown=true;}
  require(thrown,"invalid config");
  std::cout<<"PASS: 20 owned-action, negative timeout and PB-derived tree contracts\n";return 0;
 }catch(const std::exception&e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}
}
