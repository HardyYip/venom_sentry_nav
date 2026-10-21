if __name__ == "__main__":
    from storage_entry import ensure_storage_guard
    ensure_storage_guard(__file__)

import argparse
import datetime
import fcntl
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
parser.add_argument('--spin-mode',action='store_true',help='Allow translation-only stop for successful spinning arrival; safety stops remain all-zero')
parser.add_argument('--navigation-log',type=Path)
parser.add_argument('--route-set',choices=['baseline','central-marking','manual-c014'],default='baseline')
parser.add_argument('--last-case',choices=['ready','01-short','02-east-north','03-center','04-west-south','05-west-north','06-east-north-repeat','07-occupied','01-manual-first','02-manual-second','03-seam-approach','04-seam-positive-y','05-seam-negative-y'])
args=parser.parse_args()
args.last_case=args.last_case or ('05-seam-negative-y' if args.route_set=='central-marking' else '02-manual-second' if args.route_set=='manual-c014' else '07-occupied')
folder=Path(__file__).resolve().parent
workspace=folder.parents[3]
record=args.record_dir.resolve()
record.mkdir(parents=True,exist_ok=True)
lock=(workspace/'.baseline-audit.lock').open('a')
try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
except BlockingIOError:raise SystemExit('An audit already owns this workspace.')
if (record/'suite-summary.json').exists():raise SystemExit('Use a new record directory; the completed report is preserved.')
meta_path=record/'run.json'
meta=json.loads(meta_path.read_text()) if meta_path.exists() else {'started_at':datetime.datetime.now().isoformat(),'record_dir':str(record),'mode':'unchanged-current-baseline','phases':[]}
if args.navigation_log:meta['navigation_log']=str(args.navigation_log.resolve())
if not meta.get('navigation_log'):
    candidates=[]
    for candidate in (workspace/'logs').glob('*-rmul2027-nav/launches.json'):
        launch=json.loads(candidate.read_text())
        cmd=Path('/proc')/str(launch['wrapper_pid'])/'cmdline'
        log=candidate.parent/'navigation.log'
        if cmd.exists() and str(folder/'run_simulation.py').encode() in cmd.read_bytes() and log.exists():candidates.append(log)
    if len(candidates)!=1:raise SystemExit('Expected one active 2027 navigation launch; specify --navigation-log after checking runtime.')
    meta['navigation_log']=str(candidates[0])
navlog=Path(meta['navigation_log'])
meta.setdefault('navigation_log_start_byte',navlog.stat().st_size)
meta['runner_pid']=os.getpid()
active_params=navlog.parent/'nav2_params.yaml'
if not active_params.is_file():raise SystemExit('Missing active launch parameter snapshot.')
meta['mode']='current-runtime-baseline'
meta['route_set']=args.route_set
meta['active_params_sha256']=hashlib.sha256(active_params.read_bytes()).hexdigest()
meta['navigation_configuration_changed']=active_params.read_bytes()!=(folder/'config/nav2_params.yaml').read_bytes()
meta['robot_configuration_changed']=(navlog.parent/'gz_world.yaml').read_bytes()!=(folder/'config/gz_world_rounded.yaml').read_bytes()
config=record/'configuration'
config.mkdir(exist_ok=True)
contact_manifests=sorted(navlog.parent.glob('*contact-manifest.json'))
if len(contact_manifests)!=1:raise SystemExit('Expected exactly one active contact manifest.')
for source in [active_params,navlog.parent/'gz_world.yaml',folder/'behavior_trees/navigate_field.xml',folder/'scene/conversion.json',contact_manifests[0],workspace/'artifacts/rmul_2027_3v3/maps-cad/manifest.json']:
    target=config/source.name
    if not target.exists():shutil.copyfile(source,target)
rendering_snapshot=navlog.parent/'rendering-runtime.json'
if rendering_snapshot.exists():shutil.copyfile(rendering_snapshot,config/'rendering-runtime.json')
tf2_snapshot=navlog.parent/'tf2-runtime.json'
if tf2_snapshot.exists():shutil.copyfile(tf2_snapshot,config/'tf2-runtime.json')
meta['verifier_sha256']=hashlib.sha256((folder/'verify_simulation.py').read_bytes()).hexdigest()
(meta_path).write_text(json.dumps(meta,indent=2)+'\n')
stream=(record/'suite.jsonl').open('a',buffering=1)
results=[]
child=None
current_case=None
finished=False

def emit(event,**values):
    value={'event':event,'wall_epoch':time.time(),**values}
    stream.write(json.dumps(value)+'\n')
    print(json.dumps(value),flush=True)

def save_meta():meta_path.write_text(json.dumps(meta,indent=2)+'\n')

def records(path):
    result=[]
    if not path.exists():return result
    for line in path.read_text(errors='replace').splitlines():
        try:result.append(json.loads(line))
        except (json.JSONDecodeError,ValueError):pass
    return result

def cancel_owned_goal(case):
    path=record/case/'goal-ownership.json'
    if not path.exists():return
    value=json.loads(path.read_text())
    identifier=list(bytes.fromhex(value['goal_uuid']))
    request=json.dumps({'goal_info':{'goal_id':{'uuid':identifier},'stamp':{'sec':0,'nanosec':0}}})
    command=['ros2','service','call','/red_standard_robot1/'+value['action']+'/_action/cancel_goal','action_msgs/srv/CancelGoal',request]
    response=subprocess.run(command,capture_output=True,text=True,timeout=15)
    (record/case/'owned-cancel.log').write_text(response.stdout+response.stderr)
    emit('owned_goal_cancel_requested',case=case,goal_uuid=value['goal_uuid'],exit_code=response.returncode)

def interrupted(signum,frame):raise KeyboardInterrupt
signal.signal(signal.SIGTERM,interrupted)

try:
    observer_log=record/'observer'
    status_path=observer_log/'latest.json'
    observer_live=status_path.exists() and time.time()-json.loads(status_path.read_text()).get('wall_epoch',0)<6
    if not observer_live:
        if observer_log.exists():raise RuntimeError('Previous observer is stale; use a new audit directory.')
        with (record/'observer-console.log').open('w') as log:
            observer=subprocess.Popen([sys.executable,str(folder/'g1_tf_observer.py'),'--record-dir',str(observer_log),'--duration','1200'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        meta['observer_pid']=observer.pid;save_meta()
        deadline=time.monotonic()+12
        while not status_path.exists() and time.monotonic()<deadline:time.sleep(.1)
    phases=[('ready',['--mode','ready','--stability-seconds','12']),('01-short',['--mode','nav']),('02-east-north',['--mode','nav','--goal-x','9.3','--goal-y','-0.2']),('03-center',['--mode','nav','--goal-x','4.8','--goal-y','-3.0']),('04-west-south',['--mode','nav','--goal-x','-0.03','--goal-y','-5.86']),('05-west-north',['--mode','nav','--goal-x','0.0','--goal-y','-0.2']),('06-east-north-repeat',['--mode','nav','--goal-x','9.3','--goal-y','-0.2']),('07-occupied',['--mode','nav','--scenario','invalid'])]
    if args.route_set=='central-marking':
        phases=[('ready',['--mode','ready','--stability-seconds','12']),('01-manual-first',['--mode','nav','--goal-x','4.20','--goal-y','-5.71']),('02-manual-second',['--mode','nav','--goal-x','9.50','--goal-y','-5.71']),('03-seam-approach',['--mode','nav','--goal-x','5.53','--goal-y','-4.85']),('04-seam-positive-y',['--mode','nav','--goal-x','5.53','--goal-y','-4.15']),('05-seam-negative-y',['--mode','nav','--goal-x','5.53','--goal-y','-4.85'])]
    if args.route_set=='manual-c014':
        phases=[('ready',['--mode','ready','--stability-seconds','12']),('01-manual-first',['--mode','nav','--goal-x','9.675','--goal-y','-5.375']),('02-manual-second',['--mode','nav','--goal-x','-0.075','--goal-y','-0.625'])]
    if args.last_case not in [v[0] for v in phases]:raise ValueError('Last case does not belong to selected route set')
    phases=phases[:next(i for i,value in enumerate(phases) if value[0]==args.last_case)+1]
    meta['requested_last_case']=args.last_case;save_meta()
    emit('suite_started',cases=[v[0] for v in phases])
    for name,options in phases:
        current_case=name
        target=record/name;logpath=target/'verify.jsonl'
        previous=records(logpath)
        if any(v.get('event')=='PASS' for v in previous):
            results.append({'name':name,'passed':True,'reused_existing_result':True,'metrics':[v for v in previous if v.get('event') in ['result','invalid_result','localization_stability']]})
            emit('existing_case_reused',case=name);continue
        if target.exists():raise RuntimeError('Existing incomplete case preserved: '+name)
        state=None
        deadline=time.monotonic()+12
        while time.monotonic()<deadline:
            try:
                candidate=json.loads(status_path.read_text())
                footprints=candidate.get('footprints',{})
                complete=all(isinstance(footprints.get(key,{}).get('age'),(int,float)) for key in ['local_footprint','global_footprint'])
                if complete and time.time()-candidate.get('wall_epoch',0)<6:
                    state=candidate
                    break
            except (OSError,ValueError):
                pass
            time.sleep(.1)
        if state is None:raise RuntimeError('No fresh complete observer footprint data; no goal sent.')
        if any(v in [1,2,3] for v in state.get('statuses',[])):raise RuntimeError('Another navigation action is active; leave it untouched.')
        if any(v.get('age',0)>3 for v in state.get('footprints',{}).values()):raise RuntimeError('Costmap footprint stale before goal; preserve runtime.')
        target.mkdir()
        command=[sys.executable,str(folder/'verify_simulation.py'),'--startup-timeout','45','--goal-timeout','180','--stability-seconds','2','--record-dir',str(target),*options]
        if args.spin_mode:command.append('--spin-mode')
        phase={'name':name,'command':command,'started_wall_epoch':time.time()}
        with logpath.open('w') as log:
            child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            phase['pid']=child.pid;meta['active_phase']=name;meta['phases'].append(phase);save_meta();emit('case_started',case=name,pid=child.pid)
            end=time.monotonic()+300;next_status=time.monotonic()+20
            while child.poll() is None:
                if time.monotonic()>end:raise TimeoutError('Verification process timed out: '+name)
                if time.monotonic()>next_status:
                    entries=records(logpath);emit('case_running',case=name,last_event=entries[-1] if entries else None);next_status=time.monotonic()+20
                time.sleep(.2)
        entries=records(logpath)
        passed=child.returncode==0 and any(v.get('event')=='PASS' for v in entries)
        result={'name':name,'passed':passed,'exit_code':child.returncode,'metrics':[v for v in entries if v.get('event') in ['result','invalid_result','localization_stability','FAIL']]}
        results.append(result);phase['finished_wall_epoch']=time.time();phase['passed']=passed;child=None;save_meta();emit('case_finished',**result)
        if not passed:raise RuntimeError('Case failed; remaining goals not sent: '+name)
        time.sleep(2)
    finished=True
except KeyboardInterrupt:
    emit('suite_interrupted',case=current_case)
except Exception as error:
    emit('suite_failed',case=current_case,error=str(error))
finally:
    if child is not None and child.poll() is None:
        try:cancel_owned_goal(current_case)
        except Exception as error:emit('owned_cancel_error',error=str(error))
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=20)
        except subprocess.TimeoutExpired:emit('verifier_still_running',pid=child.pid)
    end_byte=navlog.stat().st_size
    with navlog.open('rb') as source:source.seek(meta['navigation_log_start_byte']);interval=source.read(end_byte-meta['navigation_log_start_byte']).decode(errors='replace')
    (record/'navigation-interval.log').write_text(interval)
    summary={'completed':finished,'route_set':args.route_set,'requested_last_case':args.last_case,'cases':results,'navigation_log_start_byte':meta['navigation_log_start_byte'],'navigation_log_end_byte':end_byte,'backup_events_in_interval':interval.count('Running backup'),'empty_plan_events_in_interval':interval.count('Resulting plan has 0 poses'),'navigation_configuration_changed':meta['navigation_configuration_changed'],'robot_configuration_changed':meta['robot_configuration_changed'],'historical_frozen_pose_root_cause_fixed':None,'root_cause_assessment':'not inferred by this route suite','finished_at':datetime.datetime.now().isoformat()}
    (record/'suite-summary.json').write_text(json.dumps(summary,indent=2)+'\n');meta['active_phase']=None;meta['suite_complete']=finished;save_meta()
    emit('suite_finished',completed=finished,cases_passed=sum(v['passed'] for v in results),cases_recorded=len(results));stream.close();lock.close()
raise SystemExit(0 if finished else 1)
