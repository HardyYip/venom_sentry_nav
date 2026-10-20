import argparse
import datetime
import hashlib
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
parser.add_argument('--params-file',type=Path,required=True)
parser.add_argument('--gz-world-path',type=Path)
parser.add_argument('--field-contact',choices=['cad','seam-ramps'],default='seam-ramps')
parser.add_argument('--last-case',default='03-center',choices=['ready','01-short','02-east-north','03-center','07-occupied'])
args=parser.parse_args()
folder=Path(__file__).resolve().parent
workspace=folder.parents[3]
record=args.record_dir.resolve()
record.mkdir(parents=True,exist_ok=False)
params=args.params_file.resolve()
if not params.is_file():raise SystemExit('Missing parameters: '+str(params))
gz_world=args.gz_world_path.resolve() if args.gz_world_path else None
if gz_world is not None and not gz_world.is_file():raise SystemExit('Missing spawn configuration: '+str(gz_world))
metadata={'owner':'Codex timing experiment','started_at':datetime.datetime.now().astimezone().isoformat(),'record_dir':str(record),'params_file':str(params),'last_case':args.last_case,'processes':{},'status':'starting','finished':False}
(record/'session.json').write_text(json.dumps(metadata,indent=2)+'\n')
metadata['field_contact']=args.field_contact
metadata['gz_world_path']=str(gz_world) if gz_world else None
metadata['params_sha256']=hashlib.sha256(params.read_bytes()).hexdigest()
if gz_world is not None:metadata['gz_world_sha256']=hashlib.sha256(gz_world.read_bytes()).hexdigest()
streams=[]
children={}

def save():
    (record/'session.json').write_text(json.dumps(metadata,indent=2)+'\n')

def launch(name,command):
    stream=(record/(name+'-console.log')).open('w')
    streams.append(stream)
    process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    children[name]=process
    metadata['processes'][name]={'pid':process.pid,'command':command}
    save()
    return process

def stop(name,timeout):
    process=children.get(name)
    if process is None or process.poll() is not None:return
    process.send_signal(signal.SIGINT)
    try:process.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        metadata.setdefault('cleanup_timeouts',[]).append(name)
        save()
        raise RuntimeError('Owned process did not stop: '+name+' pid='+str(process.pid))

def interrupt(signum,frame):raise KeyboardInterrupt
signal.signal(signal.SIGTERM,interrupt)
try:
    observer=launch('observer',[sys.executable,str(folder/'g1_tf_observer.py'),'--record-dir',str(record/'observer'),'--duration','3600','--interval','.5'])
    topics=['/clock','/red_standard_robot1/tf','/red_standard_robot1/tf_static','/red_standard_robot1/global_costmap/published_footprint','/red_standard_robot1/local_costmap/published_footprint','/red_standard_robot1/chassis_odometry_gt','/red_standard_robot1/livox/imu','/red_standard_robot1/velodyne_points','/red_standard_robot1/aft_mapped_to_init','/red_standard_robot1/odometry','/red_standard_robot1/plan','/red_standard_robot1/cmd_vel']
    recorder=launch('recorder',['ros2','bag','record','-o',str(record/'inputs'),*topics])
    command=['bash',str(folder/'start_rmul2027.sh'),'nav','--params-file',str(params),'--tf2-library-dir',str(workspace/'artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib')]
    command.extend(['--field-contact',args.field_contact])
    if gz_world is not None:command.extend(['--gz-world-path',str(gz_world)])
    wrapper=launch('launcher',command)
    deadline=time.monotonic()+180
    navlog=None
    while time.monotonic()<deadline:
        if wrapper.poll() is not None:raise RuntimeError('Simulation launcher exited before navigation ready')
        text=(record/'launcher-console.log').read_text(errors='replace')
        for line in text.splitlines():
            if 'RMUL 2027 nav started. Logs: ' in line:
                navlog=Path(line.split('Logs: ',1)[1].strip())/'navigation.log'
        if navlog is not None:break
        time.sleep(.2)
    if navlog is None:raise TimeoutError('Navigation startup exceeded180wallseconds')
    metadata['navigation_log']=str(navlog)
    for name in ['robot-red_standard_robot1.sdf','resource-hashes.json','world.sdf','gz_world.yaml','nav2_params.yaml','tf2-runtime.json','run-options.json']:
        source=navlog.parent/name
        if source.is_file():
            import shutil
            shutil.copy2(source,record/name)
    metadata['status']='auditing'
    auditdir=record/'audit'
    auditdir.mkdir()
    (auditdir/'observer').symlink_to('../observer',target_is_directory=True)
    save()
    audit=launch('audit',[sys.executable,str(folder/'run_baseline_audit.py'),'--record-dir',str(auditdir),'--navigation-log',str(navlog),'--last-case',args.last_case])
    deadline=time.monotonic()+(3000 if args.last_case=='07-occupied' else 1500)
    while audit.poll() is None:
        if wrapper.poll() is not None:raise RuntimeError('Simulation exited during audit')
        if time.monotonic()>deadline:raise TimeoutError('Audit wall deadline exceeded')
        time.sleep(.5)
    metadata['audit_exit_code']=audit.returncode
    metadata['audit_summary']=json.loads((auditdir/'suite-summary.json').read_text())
    metadata['finished']=True
    metadata['status']='audit-finished'
except KeyboardInterrupt:
    metadata['status']='interrupted'
except Exception as error:
    metadata['status']='runner-failed'
    metadata['error']=str(error)
finally:
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    signal.signal(signal.SIGTERM,signal.SIG_IGN)
    for name,timeout in [('audit',35),('recorder',25),('observer',15),('launcher',45)]:
        try:stop(name,timeout)
        except Exception as error:metadata.setdefault('cleanup_errors',[]).append(str(error))
    metadata['finished_at']=datetime.datetime.now().astimezone().isoformat()
    metadata['bag_metadata_exists']=(record/'inputs/metadata.yaml').exists()
    save()
    for stream in streams:stream.close()
if 'navigation_log' in metadata:
    navlog=Path(metadata['navigation_log'])
    lines=navlog.read_text(errors='replace').splitlines()
    timing=[]
    marker='{"event":"point_lio_timing"'
    for line in lines:
        index=line.find(marker)
        if index<0:continue
        try:timing.append(json.JSONDecoder().raw_decode(line[index:])[0])
        except ValueError:pass
    (record/'timing.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in timing))
    result={'reports':len(timing),'imu_received_total':max((x['imu_count'] for x in timing),default=0),'imu_max_gap_s':max((x['imu_max_gap_s'] for x in timing),default=None),'imu_gap_count':max((x['imu_gap_count'] for x in timing),default=0),'imu_missing_intervals_estimate':max((x['imu_missing_intervals_estimate'] for x in timing),default=0),'max_frame_processing_wall_s':max((x['scan_processing_wall_max_s'] for x in timing),default=None),'max_imu_callback_age_sim_s':max((x['imu_max_age_s'] for x in timing),default=None),'max_lidar_callback_age_sim_s':max((x['lidar_max_age_s'] for x in timing),default=None)}
    (record/'timing-summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'timing':result,'audit':metadata.get('audit_summary'),'status':metadata['status']},indent=2),flush=True)
    subprocess.run([sys.executable,str(folder/'render_baseline_audit.py'),'--record-dir',str(record/'audit'),'--output-dir',str(record/'analysis')],check=True)
raise SystemExit(0 if metadata.get('audit_exit_code')==0 and not metadata.get('cleanup_errors') else 1)
