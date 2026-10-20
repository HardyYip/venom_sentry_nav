import argparse
import json
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

parser=argparse.ArgumentParser();parser.add_argument('--record-dir',type=Path,required=True);parser.add_argument('--output-dir',type=Path,required=True);args=parser.parse_args()
root=args.record_dir;output=args.output_dir;output.mkdir(parents=True,exist_ok=True)
def read(path):return json.loads(path.read_text()) if path.exists() else None
summary={'record_directory':str(root),'scope':'Specified static simulator routes; not real robot calibration','suites':{},'runtime_guard':{},'pending':[]}
all_results=[]
for name in ['stable-full-01','stable-full-02','normal-full-01']:
 data=read(root/name/'audit/suite-summary.json')
 if not data:summary['pending'].append(name);continue
 results=[m for c in data['cases'] for m in c['metrics'] if m['event']=='result']
 summary['suites'][name]={'completed':data['completed'],'cases_passed':sum(c['passed'] for c in data['cases']),
  'motion_goals':len(results),'max_truth_error_m':max(m['ground_truth_goal_error'] for m in results),
  'max_estimate_error_m':max(m['goal_error'] for m in results),'max_stopped_component':max(m['stopped_speed'] for m in results),
  'backups':data['backup_events_in_interval'],'empty_local_paths':data['empty_plan_events_in_interval']}
 all_results.append((name,results))
for mode in ['additive','heading']:
 data=read(root/('guard-final-'+mode)/'results.json')
 summary['runtime_guard'][mode]={'passed':data['passed'],'checks':len(data['results']),'checks_detail':data['results']}
for name in ['physical-fault-01','blocked-legacy-01','stable-static-01','rmuc2025-regression']:
 data=read(root/name/'session.json');summary[name]=data or {'status':'not run'}
contract=[json.loads(line) for line in (root/'point-time-contract-03.log').read_text().splitlines() if line.startswith('{')][-1]
summary['point_time_contract']={**contract,'test_log':'point-time-contract-03.log','navigation_candidate_adopted':False}
guards=list((root/'snapshot-full-01').rglob('guard-trigger.json'))
summary['point_time_navigation_failure']=read(guards[0]) if guards else None
summary['binary_source_commit']=(root/'code-commit.txt').read_text().strip()
summary['ui_source_commit']=(root/'ui-code-commit.txt').read_text().strip() if (root/'ui-code-commit.txt').exists() else None
summary['commit_note']='binary_source_commit was built and tested; ui_source_commit added the RViz entry; this report is a later documentation commit.'
summary['ui_closure']=read(root/'ui-closure-01/session.json')
summary['clean_prepare']=read(root/'clean-prepare.json');summary['clean_build']=read(root/'clean-build.json');summary['clean_smoke']=read(root/'clean-default-smoke/session.json')
(output/'C022-summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')

physical=read(root/'physical-fault-01/observations.json');fault=summary['physical-fault-01']['fault_started_wall']
fig,axes=plt.subplots(2,1,figsize=(10,6.3),sharex=True)
for key,label,color in [('commands','runtime output','#d1495b'),('truth','actual chassis','#247ba0')]:
 rows=physical[key]
 times=np.array([v['wall']-fault for v in rows]);vel=np.array([v['value'] if key=='commands' else v['speed'] for v in rows])
 for axis,values in zip(axes,[np.linalg.norm(vel[:,:2],axis=1),vel[:,2]]):
  x=times.copy();y=values.copy();gap=np.r_[False,np.diff(x)>.3];y[gap]=np.nan;axis.plot(x,y,label=label,color=color,lw=1.6)
for axis in axes:
 for phase,color in [('lio-paused','#fed9b7'),('software-stop','#d9e8d8')]:
  times=[v['wall']-fault for v in physical['truth'] if v['phase']==phase]
  axis.axvspan(min(times),max(times),alpha=.35,color=color,label=phase)
 axis.axvline(.5,color='gray',ls=':',lw=1)
 axis.grid(alpha=.2);axis.legend(loc='upper right',fontsize=8)
axes[0].set_ylabel('Planar speed (m/s)');axes[1].set_ylabel('Angular speed (rad/s)');axes[1].set_xlabel('Wall time from Point-LIO pause (s)')
axes[0].set_title('Runtime protection stops motion and spin; fresh-input recovery')
axes[0].text(.02,.91,f"Full-zero output at {summary['physical-fault-01']['runtime_zero_delay_s']:.3f} s; simulated drift {summary['physical-fault-01']['fault_displacement_m']:.3f} m",transform=axes[0].transAxes,fontsize=9)
fig.tight_layout();fig.savefig(output/'C022-runtime-fault.png',dpi=150);plt.close(fig)

fig,axis=plt.subplots(figsize=(11,4.5));cursor=0
for name,results in all_results:
 x=np.arange(len(results))+cursor;axis.bar(x,[v['ground_truth_goal_error'] for v in results],label=name,width=.8);cursor+=len(results)+1
axis.scatter([cursor],[summary['point_time_navigation_failure']['error_m']],marker='x',s=90,c='#d1495b',label='Rejected snapshot candidate: aligned error')
axis.axhline(.35,c='#d1495b',ls='--',label='0.35 m protection / acceptance limit')
axis.set_ylim(0,.43);axis.set_ylabel('Error (m)');axis.set_xlabel('Individual goals (failure point is not endpoint error)');axis.set_title('Stable legacy input passes; snapshot candidate is not adopted');axis.legend(fontsize=8,ncol=2);axis.grid(axis='y',alpha=.2);fig.tight_layout();fig.savefig(output/'C022-goal-errors.png',dpi=150);plt.close(fig)

rows=[json.loads(line) for line in (root/'stable-static-01/samples.jsonl').read_text().splitlines()]
rows=[v for v in rows if v['phase']=='goal'];xy=np.array([v['truth_map'][:2] for v in rows])
fig,axis=plt.subplots(figsize=(8,5))
axis.plot(xy[:,0],xy[:,1],color='#247ba0',lw=2,label='Actual chassis path')
axis.add_patch(Rectangle((3-.29,-5.5-.25),.58,.50,color='#d1495b',alpha=.7,label='New static obstacle (.58 x .50 m)'))
axis.scatter([1,5],[-5.5,-5.5],c=['#247ba0','#2a9d8f'],s=70,marker='o')
axis.text(1,-5.65,'start');axis.text(5,-5.65,'goal')
axis.set_aspect('equal');axis.set_xlim(.3,5.7);axis.set_ylim(-6.3,-4.25);axis.set_xlabel('Map x (m)');axis.set_ylabel('Map y (m)')
axis.set_title('Sensed static obstacle: detour, zero measured obstacle contacts')
axis.text(.02,.04,f"Minimum footprint gap {summary['stable-static-01']['min_footprint_clearance_m']:.3f} m; removal clears both costmaps",transform=axis.transAxes,fontsize=9)
axis.legend(loc='upper left',fontsize=8);axis.grid(alpha=.2);fig.tight_layout();fig.savefig(output/'C022-static-detour.png',dpi=150);plt.close(fig)

if guards:
 guard=summary['point_time_navigation_failure']
 (output/'C022-snapshot-failure-frame.json').write_text(json.dumps(guard,indent=2)+'\n')
 trace=read(guards[0].parent/'trace.json')
 truth=sorted({v['stamp']:(v['value'][0]+4.8,v['value'][1]-3.0) for v in trace if v['kind']=='ground_truth'}.items())
 estimates=[v for v in trace if v['kind']=='estimated_pose']
 times=np.array([v[0] for v in truth]);truth_xy=np.array([v[1] for v in truth]);errors=[];stamps=[]
 for estimate in estimates:
  stamp=estimate['stamp'];index=np.searchsorted(times,stamp)
  if index<=0 or index>=len(times) or times[index]-times[index-1]>.1:
   stamps.append(stamp);errors.append(np.nan);continue
  ratio=(stamp-times[index-1])/(times[index]-times[index-1]);aligned=truth_xy[index-1]+ratio*(truth_xy[index]-truth_xy[index-1])
  stamps.append(stamp);errors.append(np.linalg.norm(np.array(estimate['value'][:2])-aligned))
 stamps=np.array(stamps);errors=np.array(errors)
 # 采样空档处断线，不把稀疏点连成跨越空档的假连续误差曲线
 sample_gaps=np.r_[False,np.diff(stamps)>.1];errors[sample_gaps]=np.nan
 fig,axes=plt.subplots(1,2,figsize=(11,4.3))
 axes[0].plot(truth_xy[:,0],truth_xy[:,1],label='Actual chassis',color='#247ba0')
 estimated=np.array([v['value'][:2] for v in estimates]);axes[0].plot(estimated[:,0],estimated[:,1],label='Estimated map pose',color='#d1495b')
 axes[0].set_aspect('equal',adjustable='datalim');axes[0].set_xlabel('Map x (m)');axes[0].set_ylabel('Map y (m)');axes[0].legend(fontsize=8)
 axes[1].plot(stamps,errors,label='Aligned position error',color='#247ba0')
 axes[1].scatter([guard['stamp']],[guard['error_m']],color='#d1495b',marker='x',s=70,label='Exact guard sample')
 axes[1].axhline(.35,color='#d1495b',ls='--',label='Unchanged 0.35 m guard')
 axes[1].set_xlabel('TF source simulation stamp (s)');axes[1].set_ylabel('Position error (m)');axes[1].set_ylim(0,max(.42,guard['error_m']+.03));axes[1].legend(fontsize=8)
 for axis in axes:axis.grid(alpha=.2)
 fig.suptitle('Rejected snapshot input: timestamp-aligned failure, no gap extrapolation')
 fig.tight_layout();fig.savefig(output/'C022-snapshot-failure.png',dpi=150);plt.close(fig)

print(json.dumps({'summary':str(output/'C022-summary.json'),'figures':4,'pending':summary['pending']}))
