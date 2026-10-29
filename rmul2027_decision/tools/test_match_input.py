#!/usr/bin/env python3
import argparse,time
from pathlib import Path
import yaml,rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from example_interfaces.msg import Bool
from rmul2027_decision.msg import MatchState

p=argparse.ArgumentParser(description='TEST state feeder; not a RMUL2027 referee protocol adapter')
p.add_argument('--input-file',type=Path,required=True)
p.add_argument('--namespace',default='/red_standard_robot1')
p.add_argument('--use-sim-time',action=argparse.BooleanOptionalAction,default=True)
a=p.parse_args();rclpy.init()
node=Node('rmul2027_test_match_input',namespace=a.namespace)
node.set_parameters([Parameter('use_sim_time',value=a.use_sim_time)])
match=node.create_publisher(MatchState,'decision/match',10)
estop=node.create_publisher(Bool,'decision/estop',1)
trust=node.create_publisher(Bool,'decision/localization_trusted',1)
try:
 while rclpy.ok():
  values=yaml.safe_load(a.input_file.read_text())
  m=MatchState();m.stamp=node.get_clock().now().to_msg();m.sequence=time.monotonic_ns()
  m.phase={'waiting':0,'running':1,'finished':2}[values['phase']]
  m.hp=float(values['hp']);m.ammo=float(values['ammo']);m.heat=float(values['heat']);m.valid=True
  m.source='TEST input file; not 2027 referee protocol'
  match.publish(m);estop.publish(Bool(data=bool(values.get('estop',True))))
  trust.publish(Bool(data=bool(values.get('localization_trusted',False))))
  rclpy.spin_once(node,timeout_sec=.05)
except KeyboardInterrupt:pass
finally:
 if rclpy.ok():
  estop.publish(Bool(data=True));trust.publish(Bool(data=False))
  node.destroy_node();rclpy.shutdown()
