from pathlib import Path
import json,os,signal,subprocess,sys,time
import argparse
parser=argparse.ArgumentParser()
parser.add_argument("--record-dir",type=Path,required=True)
parser.add_argument("--observer",type=Path,required=True)
parser.add_argument("--fixture",type=Path,required=True)
parser.add_argument("--identity",type=Path,required=True)
options=parser.parse_args()
if os.environ.get("ROS_DOMAIN_ID")!="91" or os.environ.get("IGN_PARTITION")!="codex_c054_fixture":
 parser.error("Requires private ROS_DOMAIN_ID91 and IGN_PARTITIONcodex_c054_fixture")
parent=os.environ.get("RMUL_STORAGE_PARENT_PID","")
try:guarded=bool(os.environ.get("RMUL_STORAGE_RUN_ID")) and b"storage_guard.py" in Path("/proc",parent,"cmdline").read_bytes()
except OSError:guarded=False
if not guarded:parser.error("original active capacity guard required")
P=options.record_dir.resolve()
root=Path(os.environ.get("RMUL_TEST_ARTIFACT_ROOT","/mnt/e/RMULArtifacts/pb2025_nav_ws")).resolve()
if P==root or not P.is_relative_to(root):parser.error("record dir must be beneath guarded root")
P.mkdir(parents=True,exist_ok=False)
D=Path(__file__).resolve().parents[1]
OBS=options.observer.resolve()
FIX=options.fixture.resolve()
results=[]
def execute(name,command=None,interrupt=False,gap=False,close_goal=False):
 out=P/("live-"+name)
 args=[sys.executable,"-B",str(D/"timing_capture.py"),"--output",str(out),"--observer",str(OBS),"--identity",str(options.identity.resolve()),"--seconds","12"]
 if close_goal:
  armed=P/("arm-"+name);armed.mkdir()
  (armed/"goal-ownership.json").write_text('{"goal_uuid":"00000000000000000000000000000001"}')
  args+=["--arm-root",str(armed)]
 if command:args+=["--",*command]
 with (P/(name+"-console.log")).open("xb") as log:
  proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
  if close_goal:
   time.sleep(2.)
   if close_goal=="invalid":
    (armed/"verify.jsonl").write_text('{"event":"goal_accepted","mode":"nav","goal_uuid":"00000000000000000000000000000001"}\n{"event":"invalid_result","mode":"nav","status":6}\n')
   else:
    (armed/"stop-state.json").write_text('{"original_1s_stop_passed":true}')
  if interrupt or gap:
   deadline=time.monotonic()+5
   while not (out/"processes.json").exists() and time.monotonic()<deadline:time.sleep(.05)
   identity=json.loads((out/"processes.json").read_text())
   time.sleep(3.7 if gap else .5)
   if gap:
    pid=identity["observer_pid"];os.kill(pid,signal.SIGSTOP);time.sleep(.4);os.kill(pid,signal.SIGCONT)
   else:proc.send_signal(signal.SIGTERM)
  code=proc.wait(timeout=20)
 s=json.loads((out/"summary.json").read_text())
 if name in ["healthy","closed_goal","rejected_goal"]:
  assert code==0 and s["trigger"] is None and not (out/"event.jsonl").exists(),s
  assert all(s["last_chain"]["streams"].get(k,{}).get("count",0)>0 for k in ["native_clock","native_imu","native_odometry","ros_clock","tf:map->odom","tf:odom->base_footprint"])
 elif name in ["tf_freeze","bridge_freeze","rosout","observer_gap","early_then_failure"]:
  assert code==0 and s["trigger"] and s["post_complete"] and not s["truncated"],s
  assert s["pre_available_s"]>=2.85 and s["saved_bytes"]<32*2**20,s
  reason=s["trigger"]["reason"]
  expected={"tf_freeze":"source_progress_warning","bridge_freeze":"source_progress_warning","rosout":"PoseFresh_FAILURE","observer_gap":"observer_gap","early_then_failure":"PoseFresh_FAILURE"}[name]
  assert reason==expected,(name,reason,s["trigger"])
  if name=="early_then_failure":
   assert s["primary_window_kind"]=="failure" and s["warning_window"]["trigger"]
   assert s["trigger"]["steady"]-s["warning_window"]["trigger"]["steady"]>3.
   assert any(r.get("failures") for r in map(json.loads,(out/s["event_file"]).read_text().splitlines()))
  if name=="bridge_freeze":
   rows=[json.loads(x) for x in (out/s["event_file"]).read_text().splitlines()]
   r=next(r for r in rows if r.get("type")=="chain" and r["steady"]>=s["trigger"]["steady"])
   assert r["steady"]-r["streams"]["native_clock"]["progress"]<.15
   assert r["steady"]-r["streams"]["ros_clock"]["progress"]>=.35
 elif name=="interrupted":assert code==2 and s["error"]=="interrupted",s
 elif name=="orphan":
  assert code==2 and s["error"]=="incomplete_required_header_coverage",s
  pid=int((P/"detached-grandchild.pid").read_text())
  stat=Path("/proc")/str(pid)/"stat"
  assert not stat.exists() or stat.read_text().rsplit(")",1)[1].split()[0]=="Z","grandchild leaked"
 identity=json.loads((out/"processes.json").read_text())
 for key in ["observer_pid","child_pid"]:
  pid=identity[key]
  if pid:
   stat=Path("/proc")/str(pid)/"stat"
   assert not stat.exists() or stat.read_text().rsplit(")",1)[1].split()[0]=="Z",(name,key,pid)
 resources=[]
 if s["event_file"]:
  resources=[r for r in map(json.loads,(out/s["event_file"]).read_text().splitlines()) if r.get("type")=="resource"]
  assert any(p["threads"] for r in resources for p in r["processes"]),"no per-thread resource evidence"
 results.append({"name":name,"passed":True,"exit":code,"trigger":s["trigger"],"bytes":s["saved_bytes"],
                 "pre_seconds":s["pre_available_s"],"post_seconds":s["post_observed_s"],"resource_windows":len(resources)})
 (P/"live-tests.json").write_text(json.dumps(results,indent=2)+"\n")
 print(name,"PASS",flush=True)
for mode in ["healthy","tf_freeze","bridge_freeze","rosout","early_then_failure"]:
 execute(mode,[str(FIX),mode])
execute("closed_goal",[str(FIX),"tf_freeze"],close_goal=True)
execute("rejected_goal",[str(FIX),"tf_freeze"],close_goal="invalid")
execute("observer_gap",[str(FIX),"healthy"],gap=True)
execute("interrupted",[str(FIX),"healthy"],interrupt=True)
orphan="import subprocess,pathlib,time; p=subprocess.Popen(['sleep','120'],start_new_session=True); pathlib.Path("+repr(str(P/"detached-grandchild.pid"))+").write_text(str(p.pid))"
execute("orphan",[sys.executable,"-c",orphan])
