import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import traceback
import rclpy
from rclpy.action import ActionServer,ActionClient,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from source_feed import Feed

parser=argparse.ArgumentParser();parser.add_argument('--record-dir',type=Path,required=True);args=parser.parse_args()
root=args.record_dir;root.mkdir(parents=True,exist_ok=False)
build=Path(os.environ.get('RMUL_DECISION_BUILD_ROOT','/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/decision-C062'))
binary=build/'install/rmul2027_decision/lib/rmul2027_decision/decision_node'
policy=Path(__file__).resolve().parents[1]/'config/test_policy.yaml'
rclpy.init()
results=[]

class Stub(Feed):
    def __init__(self,name,mode):
        super().__init__('/stub_'+name);self.mode=mode;self.server_rows=[];self.active=0;self.max_active=0;self.closing=False
        self.server=ActionServer(self,NavigateToPose,'navigate_to_pose',execute_callback=self.execute,
          goal_callback=self.goal,cancel_callback=self.cancel,callback_group=ReentrantCallbackGroup())
    def row(self,kind,**data):self.server_rows.append(dict(kind=kind,steady=time.monotonic(),**data))
    def goal(self,request):
        self.row('request',x=request.pose.pose.position.x)
        if self.mode=='late_accept':time.sleep(.8)
        return GoalResponse.REJECT if self.mode=='reject' else GoalResponse.ACCEPT
    def cancel(self,handle):
        self.row('cancel_request',uuid=bytes(handle.goal_id.uuid).hex())
        if self.mode=='cancel_response_delay':time.sleep(.8)
        return CancelResponse.REJECT if self.mode=='cancel_reject' else CancelResponse.ACCEPT
    def execute(self,handle):
        self.active+=1;self.max_active=max(self.max_active,self.active);start=time.monotonic()
        key=bytes(handle.goal_id.uuid).hex();self.row('execute',uuid=key)
        try:
            while not self.closing:
                if handle.is_cancel_requested and self.mode!='cancel_no_terminal':
                    handle.canceled();self.row('terminal',uuid=key,code=3);return NavigateToPose.Result()
                elapsed=time.monotonic()-start
                if self.mode!='feedback_silence':
                    f=NavigateToPose.Feedback();f.distance_remaining=max(0.,1.-elapsed);handle.publish_feedback(f)
                if self.mode=='abort' and elapsed>.12:
                    handle.abort();self.row('terminal',uuid=key,code=2);return NavigateToPose.Result()
                if self.mode=='success' and elapsed>.5:
                    handle.succeed();self.row('terminal',uuid=key,code=1);return NavigateToPose.Result()
                time.sleep(.025)
            handle.abort();return NavigateToPose.Result()
        finally:self.active-=1

def run(name,mode,scenario):
    record=root/name;record.mkdir();node=Stub(name,mode);stream=(record/'node.log').open('w')
    command=[str(binary),'--ros-args','--params-file',str(policy),'-r','__ns:='+node.get_namespace(),
      '-r','/tf:=tf','-r','/tf_static:=tf_static','-p','use_sim_time:=false','-p','recovery_hold:=0.1',
      '-p','accept_timeout:=0.35','-p','feedback_timeout:=0.35','-p','navigation_timeout:=0.8',
      '-p','cancel_timeout:=0.35','-p','retry_delay:=0.1','-p','dwell_seconds:=0.2',
      '-p','event_file:='+str(record/'events.jsonl')]
    process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);node.child=process
    result=dict(name=name,passed=False,mode=mode)
    try:
        node.wait(lambda:bool(node.status),6);node.pump(.25)
        assert not node.event('send'),'Waiting sent goal'
        node.phase=1
        scenario(node)
        assert node.max_active<=1,'Concurrent active goals'
        result.update(passed=True,max_active=node.max_active,requests=len([r for r in node.server_rows if r['kind']=='request']))
    except BaseException as exc:
        result.update(error=repr(exc),traceback=traceback.format_exc())
    finally:
        node.child=None
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=4)
        except subprocess.TimeoutExpired:process.terminate();process.wait(timeout=3)
        node.closing=True
        node.pump(.3)
        (record/'observations.json').write_text(json.dumps(dict(server=node.server_rows,status=node.status,stops=node.stops),indent=2))
        node.server.destroy();node.cleanup();stream.close()
        (record/'result.json').write_text(json.dumps(result,indent=2))
        results.append(result);(root/'results.json').write_text(json.dumps(results,indent=2))
        print(json.dumps(result),flush=True)
    if not result['passed']:raise RuntimeError('Failed '+name)

def active(n):n.wait(lambda:bool(n.event('accepted')))
def fault(expected):
    def check(n):
        n.wait(lambda:any(s['fault']==expected for s in n.status),8);count=len(n.event('send'));n.pump(.6)
        assert len(n.event('send'))==count
        assert n.status[-1]['inhibited']
    return check
def switched(n):
    active(n);n.hp=50.
    n.wait(lambda:any(x['kind']=='send' and x.get('goal',{}).get('key')=='retreat' for x in n.events),5)
    cancel=n.event('cancel')[0];old=cancel['token']
    terminal=next(x for x in n.event('action_result') if x['token']==old)
    new=next(x for x in n.event('send') if x['token']>old)
    assert terminal['steady']<=new['steady'],'New goal preceded terminal'
    n.wait(lambda:len([x for x in n.event('action_result') if x['code']==1])>=1)
    n.hp=300.
    n.wait(lambda:len([x for x in n.event('send') if x.get('goal',{}).get('key','').startswith('patrol:')])>=2)
    n.phase=2;n.wait(lambda:n.status[-1]['mode']=='finished' and not n.status[-1]['goal'])
    count=len(n.event('send'));n.phase=1;n.pump(.6);assert len(n.event('send'))==count,'Finished latch restarted'
def cancel_fault(expected):
    def check(n):
        active(n);n.hp=50.;fault(expected)(n);assert len(n.event('send'))==1
    return check
def recovery(change,restore):
    def check(n):
        active(n);start=time.monotonic();change(n)
        n.wait(lambda:any(x['kind']=='cancel' for x in n.events),3)
        n.wait(lambda:not n.status[-1]['goal'],3)
        assert any(t>start and v for t,v in n.stops)
        old=len(n.event('send'));restore(n)
        n.wait(lambda:len(n.event('send'))>old,5)
    return check

def foreign_goal(n):
    client=ActionClient(n,NavigateToPose,'navigate_to_pose')
    assert client.wait_for_server(timeout_sec=2)
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.pose.orientation.w=1.
    future=client.send_goal_async(goal)
    n.wait(lambda:future.done());assert future.result().accepted
    fault('foreign_active_goal')(n)
    assert not n.event('send') and not n.event('cancel'),'Foreign goal taken over'
    client.destroy()
def graceful_shutdown(n):
    active(n);proc=n.child;proc.send_signal(signal.SIGINT);n.child=None
    n.wait(lambda:any(x['kind']=='terminal' and x['code']==3 for x in n.server_rows),4)
    n.wait(lambda:proc.poll() is not None,4)
    assert proc.returncode==0
try:
    run('foreign_goal','hold',foreign_goal)
    run('graceful_shutdown','hold',graceful_shutdown)
    run('closed_loop','success',switched)
    run('rejected','reject',fault('rejected_exhausted'))
    run('aborted','abort',fault('aborted_exhausted'))
    run('feedback_timeout','feedback_silence',fault('execution_timeout_exhausted'))
    run('navigation_timeout','hold',fault('execution_timeout_exhausted'))
    run('cancel_rejected','cancel_reject',cancel_fault('cancel_rejected'))
    run('cancel_no_terminal','cancel_no_terminal',cancel_fault('cancel_timeout'))
    run('cancel_response_delayed','cancel_response_delay',cancel_fault('cancel_timeout'))
    run('late_accept','late_accept',lambda n:(fault('accept_timeout')(n),n.wait(lambda:bool(n.event('action_result')),3)))
    run('referee_stream_loss','hold',recovery(lambda n:setattr(n,'match_on',False),lambda n:setattr(n,'match_on',True)))
    run('localization_invalid','hold',recovery(lambda n:setattr(n,'trust',False),lambda n:setattr(n,'trust',True)))
    run('localization_stalled_stamp','hold',recovery(lambda n:setattr(n,'freeze_stamp',True),lambda n:setattr(n,'freeze_stamp',False)))
    run('odometry_stream_loss','hold',recovery(lambda n:setattr(n,'odom_on',False),lambda n:setattr(n,'odom_on',True)))
    run('tf_stream_loss','hold',recovery(lambda n:setattr(n,'tf_on',False),lambda n:setattr(n,'tf_on',True)))
    run('estop','hold',recovery(lambda n:setattr(n,'estop',True),lambda n:setattr(n,'estop',False)))
    run('estop_heartbeat_loss','hold',recovery(lambda n:setattr(n,'estop_on',False),lambda n:setattr(n,'estop_on',True)))
    run('trust_heartbeat_loss','hold',recovery(lambda n:setattr(n,'trust_on',False),lambda n:setattr(n,'trust_on',True)))
    run('referee_replay','hold',recovery(lambda n:setattr(n,'replay',True),lambda n:setattr(n,'replay',False)))
    run('referee_future','hold',recovery(lambda n:setattr(n,'source_offset',1.),lambda n:setattr(n,'source_offset',0.)))
    run('referee_invalid','hold',recovery(lambda n:setattr(n,'valid',False),lambda n:setattr(n,'valid',True)))
finally:
    rclpy.shutdown()
print('PASS: '+str(len(results))+' real ROS Action stub scenarios',flush=True)
