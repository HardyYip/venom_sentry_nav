#pragma once
#include "rmul2027_decision/core.hpp"
#include <behaviortree_cpp/bt_factory.h>

namespace rmul2027_decision {
inline BT::Tree make_tree(BT::BehaviorTreeFactory &factory,Policy &policy,Executor &executor,double &now,const std::string &path){
  factory.registerSimpleCondition("SafetyHealthy",[&](BT::TreeNode&){
    return policy.healthy(now)&&executor.fault.empty()?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
  });
  factory.registerSimpleCondition("IsGameStatus",[&](BT::TreeNode&){
    return policy.input.phase==1?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
  });
  factory.registerSimpleCondition("IsStatusOK",[&](BT::TreeNode&){
    return !policy.retreating?BT::NodeStatus::SUCCESS:BT::NodeStatus::FAILURE;
  });
  const auto task=[&](BT::TreeNode&){
    executor.request(policy.task());policy.mode=policy.retreating?"retreat":"patrol";policy.reason="running";
    return BT::NodeStatus::SUCCESS;
  };
  factory.registerSimpleAction("ExecuteMission",task);
  factory.registerSimpleAction("StopMission",[&](BT::TreeNode&){
    executor.request(std::nullopt);
    policy.mode=policy.input.phase==2?"finished":policy.input.phase==0?"waiting":"safe_stop";
    return BT::NodeStatus::SUCCESS;
  });
  return factory.createTreeFromFile(path);
}
}
