"""Real decision executable + Nav2 Action stub; no fake arrival claims."""
import argparse,json,os,signal,subprocess,time,traceback
from pathlib import Path
import rclpy
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from example_interfaces.msg import Bool
from source_feed import Feed

parser=argparse.ArgumentParser();parser.add_argument('--record-dir',type=Path,required=True);args=parser.parse_args()
root=args.record_dir;root.mkdir(exist_ok=False,parents=True)
package=Path(__file__).resolve().parents[1]
build=Path(os.environ['RMUL_DECISION_BUILD_ROOT'])
binary=build/'install/rmul2027_decision/lib/rmul2027_decision/decision_node'
rclpy.init();results=[]

class Stub(Feed):
    def __init__(self,name,behavior):
        super().__init__('/recovery_'+name);self.behavior=behavior;self.active=0;self.max_active=0;self.closing=False
        self.hb=True;self.permit=False;self.rows=[]
        self.pub=self.create_publisher(Bool,'recovery/permit',1)
        from rclpy.clock import Clock,ClockType
        self.timer=self.create_timer(.03,self.heartbeat,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.server=ActionServer(self,NavigateToPose,'navigate_to_pose',execute_callback=self.execute,
            goal_callback=lambda _:GoalResponse.ACCEPT,cancel_callback=self.cancel,callback_group=ReentrantCallbackGroup())
    def heartbeat(self):
        if self.hb:self.pub.publish(Bool(data=self.permit))
    def cancel(self,h):
        self.rows.append(dict(kind='cancel_request',uuid=bytes(h.goal_id.uuid).hex(),t=time.monotonic()))
        return CancelResponse.REJECT if self.behavior=='reject_cancel' else CancelResponse.ACCEPT
    def execute(self,h):
        self.active+=1;self.max_active=max(self.active,self.max_active)
        try:
            while not self.closing:
                if h.is_cancel_requested and self.behavior!='no_terminal':
                    h.canceled();self.rows.append(dict(kind='terminal',t=time.monotonic(),code=3))
                    return NavigateToPose.Result()
                f=NavigateToPose.Feedback();f.distance_remaining=1.;h.publish_feedback(f);time.sleep(.02)
            h.abort();return NavigateToPose.Result()
        finally:self.active-=1

def run(name,behavior,scenario):
    record=root/name;record.mkdir()
    node=Stub(name,behavior);stream=(record/'node.log').open('w')
    p=subprocess.Popen([str(binary),'--ros-args','--params-file',str(package/'config/test_policy.yaml'),
        '-r','__ns:=/recovery_'+name,'-r','/tf:=tf','-r','/tf_static:=tf_static',
        '-p','use_sim_time:=false','-p','recovery_permit_timeout:=0.3','-p','cancel_timeout:=0.45',
        '-p','event_file:='+str(record/'events.jsonl')],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    node.child=p;result=dict(name=name,passed=False)
    try:
        node.phase=1;node.wait(lambda:bool(node.status),8);node.pump(1.4)
        scenario(node)
        assert node.max_active<=1
        result.update(passed=True,max_active=node.max_active)
    except Exception as e:result.update(error=repr(e),traceback=traceback.format_exc())
    finally:
        node.child=None;p.send_signal(signal.SIGINT)
        try:p.wait(timeout=3)
        except subprocess.TimeoutExpired:p.terminate();p.wait(timeout=3)
        node.closing=True;node.pump(.3);node.timer.cancel();node.server.destroy()
        (record/'observations.json').write_text(json.dumps(dict(status=node.status,rows=node.rows,events=node.events)))
        node.cleanup();stream.close();(record/'result.json').write_text(json.dumps(result,indent=2))
        results.append(result);(root/'results.json').write_text(json.dumps(results,indent=2));print(json.dumps(result),flush=True)
    assert result['passed'],result

def release(n):n.permit=True;n.wait(lambda:bool(n.event('accepted')))
def startup(n):
    assert not n.event('send');assert n.status[-1]['inhibited']
    assert n.status[-1]['reason']=='recovery_hold_or_missing_heartbeat'
def suspend(n,loss=False):
    release(n);old=n.event('accepted')[-1]['token']
    if loss:n.hb=False
    else:n.permit=False
    n.wait(lambda:any(e['token']==old for e in n.event('action_result')),4)
    count=len(n.event('send'));n.pump(.6);assert len(n.event('send'))==count and n.status[-1]['inhibited']
    n.hb=True;n.permit=True;n.wait(lambda:len(n.event('accepted'))>=2)
    terminal=next(e for e in n.event('action_result') if e['token']==old)
    new=n.event('send')[-1];assert terminal['steady']<new['steady']
def emergency(n):
    release(n);n.estop=True;n.wait(lambda:not n.status[-1]['goal'],4)
    count=len(n.event('send'));n.permit=True;n.pump(.6);assert len(n.event('send'))==count and n.status[-1]['inhibited']
def bad_cancel(n):
    release(n);n.permit=False;n.wait(lambda:bool(n.status[-1]['fault']),4)
    count=len(n.event('send'));n.permit=True;n.pump(.7);assert len(n.event('send'))==count and n.status[-1]['inhibited']
def finished(n):
    release(n);n.phase=2;n.wait(lambda:n.status[-1]['mode']=='finished' and not n.status[-1]['goal'],4)
    count=len(n.event('send'));n.phase=1;n.permit=True;n.pump(.6);assert len(n.event('send'))==count
try:
    run('startup_missing_permit','normal',startup)
    run('hold_cancel_terminal_then_resume','normal',suspend)
    run('heartbeat_loss_cancel_then_resume','normal',lambda n:suspend(n,True))
    run('estop_cannot_be_overridden','normal',emergency)
    run('cancel_rejected_remains_latched','reject_cancel',bad_cancel)
    run('cancel_terminal_timeout_remains_latched','no_terminal',bad_cancel)
    run('finished_cannot_resume','normal',finished)
finally:rclpy.shutdown()
