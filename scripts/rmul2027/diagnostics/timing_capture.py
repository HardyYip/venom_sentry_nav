#!/usr/bin/env python3
"""Read-only bounded clock/TF flight recorder. Never publishes navigation inputs."""
from collections import deque
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time
import uuid

REQUIRED = ("native_clock", "ros_clock", "tf:map->odom", "tf:odom->base_footprint")
def encoded(row):
    return (json.dumps(row, ensure_ascii=False, separators=(",", ":"), allow_nan=False)+"\n").encode()
def write_json(path, row):
    tmp=path.with_suffix(path.suffix+".tmp")
    tmp.write_bytes(encoded(row));tmp.replace(path)
def sha(path):
    with Path(path).open("rb") as f:
        return hashlib.file_digest(f,"sha256").hexdigest() if hasattr(hashlib,"file_digest") else hashlib.sha256(f.read()).hexdigest()
def identity(paths):
    result={}
    for name,p in paths.items():
        q=Path(p)
        try:result[name]={"path":str(q),"bytes":q.stat().st_size,"sha256":sha(q)}
        except OSError as e:result[name]={"path":str(q),"missing":type(e).__name__}
    return result

class Window:
    """One event, byte-bounded RAM history and disk. Trigger only observes, never controls."""
    def __init__(self, output, pre=3., post=2., ring_bytes=8*2**20, max_bytes=32*2**20, filename="event.jsonl"):
        self.output=Path(output);self.pre=pre;self.post=post;self.filename=filename
        self.ring_limit=ring_bytes;self.max_bytes=max_bytes;self.ring=deque();self.ring_bytes=0
        self.trigger=None;self.stream=None;self.written=0;self.saved=0;self.done=False
        self.first=None;self.last=None;self.start_saved=None;self.end_saved=None
        self.pre_evictions_bytes=0;self.gaps=0;self.truncated=False;self.invalid=0
    def accept(self,row,trigger=None):
        if self.done:return
        try:
            t=float(row["steady"])
            if not math.isfinite(t):raise ValueError()
            payload=encoded(row)
            if len(payload)>1024*1024:raise ValueError("record exceeds1MiB")
        except (ValueError,KeyError,TypeError):
            self.invalid+=1;return
        if self.first is None:self.first=t
        t=max(t,self.last or t);self.last=t
        if row.get("type")=="gap" or row.get("loop_gap",0)>.15:self.gaps+=1
        if self.trigger is None:
            self.ring.append((t,payload));self.ring_bytes+=len(payload)
            while self.ring and self.ring[0][0]<t-self.pre:
                self.ring_bytes-=len(self.ring.popleft()[1])
            while self.ring and self.ring_bytes>self.ring_limit:
                self.ring_bytes-=len(self.ring.popleft()[1]);self.pre_evictions_bytes+=1
            if trigger:
                self.trigger={"steady":t,**trigger}
                self.stream=(self.output/self.filename).open("xb")
                for at,data in self.ring:self._append(at,data)
                self.ring.clear();self.ring_bytes=0
        else:self._append(t,payload)
        if self.trigger and t>=self.trigger["steady"]+self.post:self.done=True
    def _append(self,t,payload):
        if self.truncated:return
        if self.written+len(payload)>self.max_bytes:
            self.truncated=True;self.done=True;return
        self.stream.write(payload);self.written+=len(payload);self.saved+=1
        if self.start_saved is None:self.start_saved=t
        self.end_saved=t
    def finish(self):
        if self.stream:self.stream.flush();self.stream.close();self.stream=None
        return {"trigger":self.trigger,"saved_records":self.saved,"saved_bytes":self.written,
                "start_saved":self.start_saved,"end_saved":self.end_saved,
                "pre_available_s":None if self.trigger is None or self.start_saved is None else self.trigger["steady"]-self.start_saved,
                "post_observed_s":None if self.trigger is None or self.end_saved is None else self.end_saved-self.trigger["steady"],
                "post_complete":bool(self.trigger and self.end_saved is not None and self.end_saved-self.trigger["steady"]>=self.post),
                "truncated":self.truncated,"memory_limit_evictions":self.pre_evictions_bytes,
                "observation_gaps":self.gaps,"invalid_records":self.invalid,
                "limits":{"pre_seconds":self.pre,"post_seconds":self.post,"ring_bytes":self.ring_limit,"event_bytes":self.max_bytes}}

def active_owned_goal(root):
    for owner in root.rglob("goal-ownership.json"):
        if (owner.parent/"stop-state.json").exists():
            continue
        terminal=False
        try:
            identifier=json.loads(owner.read_text()).get("goal_uuid")
            trace=owner.parent/"verify.jsonl"
            with trace.open("rb") as stream:
                stream.seek(0,2)
                stream.seek(max(0,stream.tell()-65536))
                lines=stream.read().splitlines()
            accepted=False
            for line in lines:
                try:row=json.loads(line)
                except (ValueError,TypeError):continue
                if row.get("event")=="goal_accepted":
                    accepted=bool(identifier and row.get("goal_uuid")==identifier)
                    terminal=False
                elif accepted and row.get("event")=="invalid_result" and row.get("mode")=="nav" and row.get("status")==6:
                    terminal=True
        except (OSError,ValueError,TypeError,AttributeError):
            pass
        if not terminal:
            return True
    return False

def trigger_for(row, armed):
    failures=row.get("failures",[])
    if failures:return {"reason":"PoseFresh_FAILURE","observations":failures[:4],"causality":"not_determined"}
    streams=row.get("streams",{})
    if not armed or not all(streams.get(k,{}).get("count",0)>0 for k in REQUIRED):return None
    if row.get("loop_gap",0)>.15:
        return {"reason":"observer_gap","seconds":row["loop_gap"],"causality":"uncovered_observation"}
    stalls={k:row["steady"]-streams[k]["progress"] for k in REQUIRED
            if row["steady"]-streams[k]["progress"]>=.35}
    if stalls:
        return {"reason":"source_progress_warning","progress_age_s":stalls,"causality":"not_determined"}
    return None

def observed_process(command):
    if command.startswith(("ign gazebo ","ruby /usr/bin/ign gazebo ")):
        return True
    words=command.split()
    return bool(words and Path(words[0]).name in {
        "parameter_bridge","component_container","component_container_isolated",
        "pointlio_mapping","timing_chain_observer"})

def resource_snapshot():
    begin=time.monotonic();rows=[];clipped=False
    for p in Path("/proc").iterdir():
        if not p.name.isdigit():continue
        try:
            cmd=(p/"cmdline").read_bytes().replace(b"\0",b" ").decode(errors="replace")
            if not observed_process(cmd):continue
            stat=(p/"stat").read_text().rsplit(")",1)[1].split()
            threads=[];thread_read_errors=0
            for k,d in enumerate((p/"task").iterdir()):
                if k>=128:clipped=True;break
                try:
                    st=(d/"stat").read_text().rsplit(")",1)[1].split()
                    threads.append({"tid":int(d.name),"comm":(d/"comm").read_text().strip()[:64],
                      "start_ticks":int(st[19]),"utime":int(st[11]),"stime":int(st[12]),
                      "schedstat":[int(x) for x in (d/"schedstat").read_text().split()]})
                except (OSError,ValueError,IndexError):thread_read_errors+=1
            rows.append({"pid":int(p.name),"cmd":cmd[:512],"start_ticks":int(stat[19]),
              "utime":int(stat[11]),"stime":int(stat[12]),"rss_pages":int(stat[21]),
              "declared_threads":int(stat[17]),"thread_read_errors":thread_read_errors,"threads":threads})
            if len(rows)>=32:clipped=True;break
        except (OSError,ValueError,IndexError):pass
    pressure={}
    for key in ["cpu","memory","io"]:
        try:pressure[key]=(Path("/proc/pressure")/key).read_text()
        except OSError:pressure[key]=None
    return {"type":"resource","steady":begin,"wall":time.time(),"collection_seconds":time.monotonic()-begin,
            "clock_ticks_per_sec":os.sysconf("SC_CLK_TCK"),"page_size":os.sysconf("SC_PAGE_SIZE"),
            "processes":rows,"pressure":pressure,"clipped":clipped,
            "limit":"all observable threads, at most32processes and128threads/process; absent or clipped data is not proof of no starvation"}

def owned_launch(command,**kwargs):
    token=uuid.uuid4().hex
    env=dict(os.environ,RMUL_CAPTURE_OWNER=token)
    process=subprocess.Popen(command,env=env,start_new_session=True,**kwargs)
    process._capture_owner=token
    try:process._capture_start_ticks=Path("/proc",str(process.pid),"stat").read_text().rsplit(")",1)[1].split()[19]
    except OSError:process._capture_start_ticks=None
    return process

def stop_group(process):
    if not process:return
    token=getattr(process,"_capture_owner",None)
    if not token:raise RuntimeError("missing capture ownership")
    marker=("RMUL_CAPTURE_OWNER="+token).encode()
    def targets():
        result=[]
        for p in Path("/proc").iterdir():
            if not p.name.isdigit():continue
            try:
                if marker not in (p/"environ").read_bytes().split(bytes([0])):continue
                st=(p/"stat").read_text().rsplit(")",1)[1].split()
                if st[0]!="Z":result.append((int(p.name),st[19]))
            except (OSError,ValueError,IndexError):pass
        return result
    for sig,seconds in [(signal.SIGINT,3),(signal.SIGTERM,2),(signal.SIGKILL,1)]:
        current=targets()
        if not current:break
        for pid,start_ticks in current:
            try:
                st=Path("/proc",str(pid),"stat").read_text().rsplit(")",1)[1].split()
                if st[19]==start_ticks:os.kill(pid,sig)
            except (OSError,ValueError,IndexError):pass
        deadline=time.monotonic()+seconds
        while targets() and time.monotonic()<deadline:time.sleep(.03)
    try:process.wait(timeout=1)
    except subprocess.TimeoutExpired:raise RuntimeError("owned process did not stop")
    if targets():raise RuntimeError("owned descendants still alive")

def run(args):
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    stop=threading.Event();inbox=queue.Queue(maxsize=128);errors=deque(maxlen=16)
    dropped=[0];observer=None;child=None;final_code=2;failure=None;resources=0;records=0;last_chain=None
    paths=json.loads(args.identity.read_text()) if args.identity else {}
    if len(paths)>64:raise ValueError("identity manifest exceeds64paths")
    paths["capture_observer_binary"]=str(args.observer.resolve())
    paths["capture_python_source"]=str(Path(__file__).resolve())
    before=identity(paths);start=time.monotonic()
    write_json(output/"identity-start.json",{"at":datetime.datetime.now().astimezone().isoformat(),"files":before})
    window=Window(output,pre=args.pre,post=args.post)
    warning=Window(output,pre=args.pre,post=args.post,ring_bytes=4*2**20,max_bytes=8*2**20,filename="warning.jsonl")
    def accept(row,trigger=None):
        fatal=bool(trigger and trigger["reason"]=="PoseFresh_FAILURE")
        window.accept(row,trigger if fatal else None)
        warning.accept(row,trigger if trigger and not fatal else None)
    def interrupt(sig,frame):stop.set()
    old_handlers={s:signal.signal(s,interrupt) for s in (signal.SIGINT,signal.SIGTERM)}
    def reader(stream):
        try:
            for line in iter(stream.readline,b""):
                if len(line)>1024*1024:dropped[0]+=1;continue
                try:
                    row=json.loads(line);row["type"]="chain"
                    inbox.put_nowait(row)
                except queue.Full:dropped[0]+=1
                except (ValueError,TypeError):
                    dropped[0]+=1;errors.append(line[:2048].decode(errors="replace"))
        except (OSError,ValueError) as e:errors.append(repr(e))
    def stderr_reader(stream):
        for line in iter(stream.readline,b""):errors.append(line[:2048].decode(errors="replace"))
    try:
        observer=owned_launch([str(args.observer.resolve()),"/dev/stdout"],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
        read_thread=threading.Thread(target=reader,args=(observer.stdout,),daemon=True);read_thread.start()
        err_thread=threading.Thread(target=stderr_reader,args=(observer.stderr,),daemon=True);err_thread.start()
        command=args.command[1:] if args.command[:1]==["--"] else args.command
        if command:child=owned_launch(command)
        write_json(output/"processes.json",{"observer_pid":observer.pid,"child_pid":None if not child else child.pid,
          "command":command,"read_only_observer":True,"started_steady":start,
          "observer_start_ticks":observer._capture_start_ticks,"observer_owner_token":observer._capture_owner,
          "child_start_ticks":None if not child else child._capture_start_ticks,"child_owner_token":None if not child else child._capture_owner})
        next_resource=start;next_arm=start;armed=not args.arm_root;last_dropped=0
        while not stop.is_set() and time.monotonic()-start<args.seconds:
            now=time.monotonic()
            if now>=next_arm:
                if args.arm_root:armed=active_owned_goal(args.arm_root)
                next_arm=now+.25
            if now>=next_resource:
                accept(resource_snapshot());resources+=1;next_resource=now+1.
            try:row=inbox.get(timeout=.03)
            except queue.Empty:row=None
            if dropped[0]!=last_dropped:
                accept({"type":"gap","steady":now,"wall":time.time(),"dropped_total":dropped[0]})
                last_dropped=dropped[0]
            if row:
                records+=1;last_chain=row;accept(row,trigger_for(row,armed))
            if observer.poll() is not None:
                failure="observer_exited";break
            if child and child.poll() is not None and (not window.trigger or window.done):break
            if window.done and not child:break
        if stop.is_set():failure="interrupted"
        elif child and child.poll() is None and time.monotonic()-start>=args.seconds:failure="duration_limit"
        final_code=2 if failure else (child.returncode if child and child.returncode is not None else 0)
    except Exception as e:failure=repr(e);final_code=2
    finally:
        stop_group(child);stop_group(observer)
        if observer:
            read_thread.join(timeout=1);err_thread.join(timeout=1)
        fatal_result=window.finish();warning_result=warning.finish()
        result=dict(fatal_result if fatal_result["trigger"] else warning_result)
        result["primary_window_kind"]="failure" if fatal_result["trigger"] else ("warning" if warning_result["trigger"] else "none")
        result["warning_window"]=warning_result.copy()
        result["event_file"]="event.jsonl" if fatal_result["trigger"] else ("warning.jsonl" if warning_result["trigger"] else None)
        result["total_event_bytes"]=fatal_result["saved_bytes"]+warning_result["saved_bytes"]
        missing=[k for k in REQUIRED if not last_chain or last_chain.get("streams",{}).get(k,{}).get("count",0)==0]
        if not failure and missing:
            failure="incomplete_required_header_coverage";final_code=2
        result["required_streams_missing"]=missing
        result["last_chain_age_at_end_s"]=None if not last_chain else time.monotonic()-last_chain["steady"]
        result["data_quality_complete"]=not missing and not result["observation_gaps"] and not result["invalid_records"] and not result["truncated"] and not dropped[0] and not (last_chain and (last_chain.get("stream_overflow",0) or last_chain.get("event_overflow",0)))
        after=identity(paths)
        write_json(output/"identity-end.json",{"at":datetime.datetime.now().astimezone().isoformat(),"files":after})
        runtime=[]
        if args.arm_root and args.arm_root.exists():
            for session in list(args.arm_root.rglob("session.json"))[:16]:
                try:
                    meta=json.loads(session.read_text());nav=Path(meta["navigation_log"]).parent
                    entry={"session":str(session),"session_sha256":sha(session),"navigation_log":str(nav/"navigation.log"),"runtime_files":{}}
                    for name in ["dependency-runtime.json","dependency-runtime-end.json","tf2-runtime.json","run-options.json"]:
                        q=nav/name
                        if q.is_file() and q.stat().st_size<=512*1024:
                            raw=q.read_bytes();target=output/(str(len(runtime))+"-"+name);target.write_bytes(raw)
                            entry["runtime_files"][name]={"sha256":hashlib.sha256(raw).hexdigest(),"copy":str(target)}
                    runtime.append(entry)
                except (OSError,ValueError,KeyError):pass
        result.update({"at":datetime.datetime.now().astimezone().isoformat(),"phase":"complete" if not failure else "incomplete",
         "error":failure,"exit_code":final_code,"observed_chain_records":records,"resource_samples":resources,
         "dropped_pipe_records":dropped[0],"last_chain":last_chain,"observer_stderr_tail":list(errors),"identity_unchanged":before==after,
         "observer_exit":None if not observer else observer.returncode,"child_exit":None if not child else child.returncode,
         "runtime_sources":runtime,"wall_duration_s":time.monotonic()-start,"historical_root_identified":False,
         "no_event_means":"No event observed in this bounded run; no proof of historical fix",
         "causality_limit":"Native receipt gaps may be publisher/transport/observer scheduling; thread samples are1Hz snapshots, not continuous scheduling traces"})
        if result["event_file"]:result["event_sha256"]=sha(output/result["event_file"])
        write_json(output/"summary.json",result)
        for sig,handler in old_handlers.items():signal.signal(sig,handler)
    return final_code

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output","--record-dir",dest="output",type=Path,required=True);p.add_argument("--observer",type=Path,required=True)
    p.add_argument("--identity",type=Path);p.add_argument("--arm-root",type=Path)
    p.add_argument("--seconds",type=float,default=240.);p.add_argument("--pre",type=float,default=3.)
    p.add_argument("--post",type=float,default=2.);p.add_argument("command",nargs=argparse.REMAINDER)
    args=p.parse_args()
    if not (0<args.seconds<=1800 and 0<args.pre<=5 and 0<args.post<=5):p.error("bounded seconds<=1800, pre/post<=5")
    root=Path(os.environ.get("RMUL_TEST_ARTIFACT_ROOT","/mnt/e/RMULArtifacts/pb2025_nav_ws")).resolve()
    if args.output.resolve()==root or not args.output.resolve().is_relative_to(root):
        p.error("output must be a new directory beneath the guarded artifact root")
    parent=os.environ.get("RMUL_STORAGE_PARENT_PID","")
    try:guarded=bool(os.environ.get("RMUL_STORAGE_RUN_ID")) and b"storage_guard.py" in Path("/proc",parent,"cmdline").read_bytes()
    except OSError:guarded=False
    if not guarded:p.error("active original storage_guard required; use scripts/rmul2027/run_timing_capture.py")
    return run(args)
if __name__=="__main__":raise SystemExit(main())
