"""Actual ROS executors: main-work isolation and a real input gap."""
import argparse,json,os,threading,time,sys
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"recovery"))
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from inputs import InputReader
from supervisor import Supervisor
from imu_relay import ImuRelay
p=argparse.ArgumentParser();p.add_argument("--record-dir",type=Path,required=True);p.add_argument("--with-relay",action="store_true");a=p.parse_args()
assert os.environ.get("RMUL_STORAGE_RUN_ID") and os.environ.get("ROS_DOMAIN_ID")=="91"
a.record_dir.mkdir(exist_ok=False,parents=True);result=dict(passed=False,checks=[])
rclpy.init();publisher=rclpy.create_node("input_reader_fixture",namespace="/red_standard_robot1")
cp=publisher.create_publisher(Clock,"/clock",qos_profile_sensor_data)
ip=publisher.create_publisher(Imu,"livox/imu",qos_profile_sensor_data)
relay=ImuRelay("/red_standard_robot1") if a.with_relay else None
reader=InputReader("recovery/test_imu" if relay else "livox/imu");running=True;imu_on=True
def source():
    started=time.monotonic();last=-1
    while running:
        t=time.monotonic()-started;ns=int((20+t)*10**9)
        c=Clock();c.clock.sec,c.clock.nanosec=divmod(ns,10**9);cp.publish(c)
        slot=int(t/.005)
        if imu_on and slot!=last:
            m=Imu();m.header.stamp=c.clock;m.linear_acceleration.z=9.81;ip.publish(m)
        last=slot;time.sleep(.001)
thread=threading.Thread(target=source);thread.start()
def healthy():
    s=reader.window.snapshot()
    proxy=SimpleNamespace(imu=s['samples'],imu_at=s['at'],imu_stamp=s['stamp'],imu_since=s['since'],ros=reader.ros)
    return bool(Supervisor.raw_healthy(proxy,time.monotonic()))
def wait(pred,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end and not pred():time.sleep(.005)
    assert pred()
def check(name,**kw):result['checks'].append(dict(name=name,passed=True,**kw))
try:
    wait(healthy,8);check("actual_source_clock_and_imu_healthy")
    begin=time.monotonic();time.sleep(.35)
    assert healthy()
    invalid=[x for x in reader.window.snapshot()['invalid'] if x['steady']>=begin]
    assert not invalid
    check("blocked_main_does_not_block_input_reader",main_block_wall_s=time.monotonic()-begin)
    imu_on=False;time.sleep(.2);assert not healthy();check("real_input_gap_is_not_healthy")
    resumed=time.monotonic();imu_on=True;time.sleep(.15);assert not healthy()
    assert any(x['steady']>=resumed for x in reader.window.snapshot()['invalid'])
    check("gap_source_stamps_reset_two_second_continuity")
    wait(healthy,5);check("two_seconds_new_real_inputs_restore_health")
    result['passed']=True
finally:
    running=False;thread.join(timeout=3);reader.close()
    assert not reader.thread.is_alive() and not thread.is_alive()
    if relay:
        result["relay"]=relay.snapshot();relay.close();assert not relay.thread.is_alive()
    check("owned_input_thread_cleanup")
    publisher.destroy_node();rclpy.shutdown()
    (a.record_dir/"result.json").write_text(json.dumps(result,indent=2))
print(json.dumps(result))
