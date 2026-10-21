import argparse
import json
from pathlib import Path
import re
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
parser.add_argument('--output-dir',type=Path,required=True)
parser.add_argument('--continuation-dir',type=Path)
args=parser.parse_args();root=args.record_dir;out=args.output_dir;out.mkdir(parents=True,exist_ok=True)
def read(path,default=None):
    return json.loads(path.read_text()) if path.exists() else default
summary={'record_directory':str(root),'scope':'Polar Bear static simulator navigation and structural fault handling; not Venom hardware','tests':{},'suites':{},'failed_attempts':[]}
for name in ['watchdog-01','watchdog-02','watchdog-03','localization-01','localization-02','localization-03','localization-04','localization-relay-01','localization-relay-02','lio-gap-01','lio-gap-02','lio-gap-03','lio-buffered-01']:
    data=read(root/name/'session.json')
    if data is None:continue
    value={k:data[k] for k in ['passed','checks','error','scenario','runtime_hashes','cleanup_pending'] if k in data}
    if data.get('passed'):summary['tests'][name]=value
    else:summary['failed_attempts'].append({'name':name,**value})
for folder in sorted(root.glob('*full-*')):
    data=read(folder/'audit/suite-summary.json')
    if data is None:continue
    results=[m for c in data['cases'] for m in c.get('metrics',[]) if m.get('event')=='result']
    summary['suites'][folder.name]={'completed':data['completed'],'cases_passed':sum(c['passed'] for c in data['cases']),'motion_goals':len(results),'max_truth_error_m':max((v['ground_truth_goal_error'] for v in results),default=None),'max_estimate_error_m':max((v['goal_error'] for v in results),default=None),'max_stopped_component':max((v['stopped_speed'] for v in results),default=None),'backups':data.get('backup_events_in_interval'),'empty_local_paths':data.get('empty_plan_events_in_interval'),'goals':results,'dependency_runtime':read(folder/'dependency-runtime.json')}
for name in ['snapshot-static-01','snapshot-blocked-01','rmuc2025-regression']:
    data=read(root/name/'session.json')
    if data is not None:summary['tests'][name]=data
if args.continuation_dir:
    continuation=args.continuation_dir
    summary['continuation_record_directory']=str(continuation)
    data=read(continuation/'normal-full-01/audit/suite-summary.json')
    if data:
        results=[m for c in data['cases'] for m in c.get('metrics',[]) if m.get('event')=='result']
        summary['suites']['C025-normal-full-01']={'completed':data['completed'],'cases_passed':sum(c['passed'] for c in data['cases']),'motion_goals':len(results),'max_truth_error_m':max((v['ground_truth_goal_error'] for v in results),default=None),'max_estimate_error_m':max((v['goal_error'] for v in results),default=None),'max_stopped_component':max((v['stopped_speed'] for v in results),default=None),'backups':data.get('backup_events_in_interval'),'empty_local_paths':data.get('empty_plan_events_in_interval'),'goals':results,'dependency_runtime':read(continuation/'normal-full-01/dependency-runtime.json')}
    for name in ['static-01','blocked-01','rmuc2025-regression']:
        data=read(continuation/name/'session.json')
        if data is not None:summary['tests']['C025-'+name]=data
    summary['failed_attempts'].append({'name':'snapshot-normal-full-01','reason':'Historical session interrupted by host disk exhaustion; superseded by C025-normal-full-01, not counted as passed.'})
    summary['historical_sequence_is_not_current_status']=True
before=[json.loads(line) for line in (root/'kalman-before.jsonl').read_text().splitlines()]
after=[json.loads(line) for line in (root/'kalman-after.jsonl').read_text().splitlines()]
summary['kalman_contract']={'before_pass':sum(v['passed'] for v in before),'before_fail':sum(not v['passed'] for v in before),'after_pass':sum(v['passed'] for v in after),'checks':len(after),'maximum_state_error_after':max(v['state_error'] for v in after),'maximum_covariance_error_after':max(v['covariance_error'] for v in after)}
text=(root/'contract-clock.log').read_text();counts=re.findall(r'CHECKS=(\d+)',text)
summary['bt_contract']={'checks':int(counts[-1]) if counts else 0,'passed':bool(counts) and 'FAIL\n' not in text}
summary['patch_restoration']=read(root/'patch-verification.json')
summary['sequence']=read(root/'final-sequence.json')
summary['lost_imu_history_before_latch']=read(root/'localization-04/guard-trigger.json')
(root/'C024-summary.json').write_text(json.dumps(summary,indent=2)+'\n');(out/'C024-summary.json').write_text(json.dumps(summary,indent=2)+'\n')
fig,ax=plt.subplots(figsize=(9,4.5))
for data,label,color in [(before,'Before: inconsistent information weighting','#c94444'),(after,'After: equivalent Kalman update','#007f75')]:
    rows=[v for v in data if v['state_dof']==30 and v['variance']==.001]
    ax.semilogy([v['measurements'] for v in rows],[max(v['state_error'],1e-16) for v in rows],'o-',label=label,color=color)
ax.axvline(30,color='gray',ls='--',label='Information-form branch begins')
ax.set_xlabel('Measurements in one same-time batch');ax.set_ylabel('State-correction L2 difference from reference')
ax.set_title('Actual Point-LIO filter: same variance must give the same update');ax.grid(alpha=.2);ax.legend(fontsize=8);fig.tight_layout();fig.savefig(out/'C024-kalman-equivalence.png',dpi=160);plt.close(fig)
name='watchdog-03' if (root/'watchdog-03/observations.json').exists() else 'watchdog-02'
data=read(root/name/'observations.json')
if data:
    rows=data['truth'];origin=rows[0]['wall'];fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    for phase in ['before-freeze','container-frozen','container-resumed-no-command','fresh-commands','bridge-frozen','bridge-exited']:
        values=[v for v in rows if v['phase']==phase][::8]
        if not values:continue
        stamps=[v['wall']-origin for v in values]
        axes[0].plot(stamps,[np.hypot(*v['speed'][:2]) for v in values],label=phase)
        axes[1].plot(stamps,[abs(v['speed'][2]) for v in values],label=phase)
    axes[0].set_ylabel('Ground-truth planar speed (m/s)');axes[1].set_ylabel('Absolute yaw speed (rad/s)');axes[1].set_xlabel('Wall time since probe began (s)')
    axes[0].legend(fontsize=7,ncol=2);axes[0].set_title('Independent downstream stops; upstream guard cannot send zero while frozen')
    for ax in axes:ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(out/'C024-independent-stops.png',dpi=160);plt.close(fig)
fig,ax=plt.subplots(figsize=(10,4.5));offset=0
for name in ['snapshot-fast-full-02','snapshot-fast-full-03','C025-normal-full-01']:
    data=summary['suites'].get(name)
    if not data:continue
    rows=data['goals'];x=np.arange(len(rows))+offset
    ax.plot(x,[v['ground_truth_goal_error'] for v in rows],'o-',label=name);offset+=len(rows)+1
ax.axhline(.35,color='#c94444',ls='--',label='Unchanged 0.35 m endpoint limit');ax.set_ylim(bottom=0);ax.set_xlabel('Individual completed goals');ax.set_ylabel('Ground-truth endpoint error (m)');ax.set_title('Final snapshot configuration: fresh launches and physical stopping');ax.legend(fontsize=8);ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/'C024-navigation-errors.png',dpi=160);plt.close(fig)
print(json.dumps({'suites':{n:{k:v for k,v in x.items() if k not in ['goals','dependency_runtime']} for n,x in summary['suites'].items()},'test_names':list(summary['tests']),'kalman_contract':summary['kalman_contract'],'bt_contract':summary['bt_contract']}))
