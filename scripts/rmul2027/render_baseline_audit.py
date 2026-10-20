import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
parser.add_argument('--output-dir',type=Path)
args=parser.parse_args()
record=args.record_dir.resolve()
output=(args.output_dir or record).resolve()
output.mkdir(parents=True,exist_ok=True)
workspace=Path(__file__).resolve().parents[4]
map_file=workspace/'artifacts/rmul_2027_3v3/maps-cad/rmul_2027_3v3.yaml'
config=yaml.safe_load(map_file.read_text())
image=np.asarray(Image.open(map_file.parent/config['image']))
extent=[config['origin'][0],config['origin'][0]+image.shape[1]*config['resolution'],config['origin'][1],config['origin'][1]+image.shape[0]*config['resolution']]
fig,axes=plt.subplots(1,2,figsize=(14,6),gridspec_kw={'width_ratios':[1.25,1]})
axes[0].imshow(image,cmap='gray',vmin=0,vmax=255,extent=extent,origin='upper')
axes[0].set(xlabel='Map X (m)',ylabel='Map Y (m)',title='Measured ground-truth routes on the CAD-derived map')
axes[0].set_aspect('equal')
axes[1].axhline(.35,color='red',linestyle='--',label='0.35 m verification threshold')
axes[1].set(xlabel='Simulation seconds from first recorded sample in each case',ylabel='Timestamp-aligned XY localization error (m)',title='Localization error: estimate vs interpolated ground truth')
metrics=[]
for index,case in enumerate(sorted(p for p in record.iterdir() if p.is_dir() and (p/'trace.json').exists())):
    trace=json.loads((case/'trace.json').read_text());truth=[v for v in trace if v['kind']=='ground_truth'];est=[v for v in trace if v['kind']=='estimated_pose']
    if len(truth)<2:continue
    truth.sort(key=lambda v:v['stamp']);t=np.array([v['stamp'] for v in truth]);xyz=np.array([v['value'][:2] for v in truth])+[4.8,-3.]
    rows=[]
    for v in est:
        stamp=v['stamp']
        if not t[0]<=stamp<=t[-1]:continue
        ref=np.array([np.interp(stamp,t,xyz[:,i]) for i in range(2)])
        rows.append([stamp,float(np.linalg.norm(np.array(v['value'][:2])-ref))])
    guards=[v for v in trace if v['kind']=='localization_guard']
    rows.extend([v['stamp'],v['error_m']] for v in guards)
    if rows:
        values=np.unique(np.array(rows),axis=0)
        plot_t=[];plot_error=[]
        previous=None
        for stamp,error in values:
            if previous is not None and stamp-previous>.25:
                plot_t.append(float('nan'));plot_error.append(float('nan'))
            plot_t.append(stamp-t[0]);plot_error.append(error);previous=stamp
        axes[1].plot(plot_t,plot_error,label=case.name,linewidth=1.2,marker='.',markersize=2)
        for guard in guards:
            axes[1].scatter(guard['stamp']-t[0],guard['error_m'],marker='x',s=35,color='red',zorder=5)
    else:values=np.empty((0,2))
    events=[]
    for line in (case/'verify.jsonl').read_text(errors='replace').splitlines():
        try:events.append(json.loads(line))
        except ValueError:pass
    passed=any(v.get('event')=='PASS' for v in events)
    result=next((v for v in events if v.get('event') in ['result','invalid_result']),None)
    fail=next((v for v in events if v.get('event')=='FAIL'),None)
    length=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum())
    if case.name!='ready':
        line=axes[0].plot(xyz[:,0],xyz[:,1],linewidth=1.6,label=case.name+(' PASS' if passed else ' CANCELED/FAIL'))[0]
        axes[0].scatter(xyz[0,0],xyz[0,1],marker='o',s=16,color=line.get_color());axes[0].scatter(xyz[-1,0],xyz[-1,1],marker='x',s=28,color=line.get_color())
    metrics.append({'case':case.name,'passed':passed,'path_length_gt_m':length,'max_aligned_localization_error_m':float(values[:,1].max()) if len(values) else None,'p95_aligned_localization_error_m':float(np.quantile(values[:,1],.95)) if len(values) else None,'result':result,'failure':fail})
axes[0].legend(fontsize=7,loc='upper left');axes[1].legend(fontsize=7);axes[1].grid(alpha=.25)
fig.suptitle(record.name+' | Route and timestamp-aligned localization evidence',fontsize=10)
fig.tight_layout();fig.savefig(output/'baseline-audit.png',dpi=160);plt.close(fig)
observer_path=record/'observer/samples.jsonl'
observer={}
if observer_path.exists():
    samples=[]
    for line in observer_path.read_text().splitlines():
        try:samples.append(json.loads(line))
        except ValueError:pass
    active=[v for v in samples if 2 in v.get('statuses',[]) and v.get('plan') and -.2<=v['sim_time']-v['plan']['stamp']<=1.5 and v.get('plan_start_robot_distance_m') is not None]
    observer={'sample_count':len(samples),'plan_count':samples[-1]['counts'].get('plan',0) if samples else 0,'max_active_fresh_plan_start_distance_m':max((v['plan_start_robot_distance_m'] for v in active),default=None),'active_plan_samples':len(active),'max_footprint_ages_s':{key:max((v.get('footprints',{}).get(key,{}).get('age',0) or 0 for v in samples),default=None) for key in ['global_footprint','local_footprint']}}
result={'record_dir':str(record),'world_to_map_translation_xy_m':[4.8,-3.0],'timestamp_alignment':'interpolate recorded ground truth at each TF estimate timestamp; no ground truth is used by navigation','cases':metrics,'observer':observer,'historical_root_cause_fixed':None,'root_cause_assessment':'not inferred by this route report'}
(output/'analysis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'output_dir':str(output),'cases':[{'case':v['case'],'passed':v['passed'],'max_error':v['max_aligned_localization_error_m']} for v in metrics],'observer':observer},indent=2))
