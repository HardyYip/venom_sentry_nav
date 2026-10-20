import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
import numpy as np

parser=argparse.ArgumentParser();parser.add_argument('--record-dir',type=Path,required=True);parser.add_argument('--output-dir',type=Path,required=True)
args=parser.parse_args();root=args.record_dir;out=args.output_dir;out.mkdir(parents=True,exist_ok=True)


def rows(path):
    values=[]
    if path.exists():
        for line in path.read_text().splitlines():
            try:values.append(json.loads(line))
            except json.JSONDecodeError:pass
    return values


def metrics(case):
    path=case/'verify.jsonl'
    if not path.exists():path=case.parent/(case.name+'-console.log')
    return [v for v in rows(path) if v.get('event') in ['result','FAIL','invalid_result']]


speed=[]
names=['original-speed-01','original-speed-confirm','original-drive-original-speed-01','early-brake-01','early-brake-full-01','early-brake-full-02']
labels=['Current drive\n1m brake #1','Current drive\n1m brake #2','Original drive\n1m brake','Current drive\n2m pilot','Current drive\n2m full #1','Current drive\n2m full #2']
for name,label in zip(names,labels):
    case=root/name/'audit/02-east-north';values=metrics(case);result=next((v for v in values if v.get('event')=='result'),None)
    if result:
        trace=json.loads((case/'trace.json').read_text());commands=[v for v in trace if v.get('kind')=='cmd_vel'];peak=max((math.hypot(*v['value'][:2]) for v in commands),default=0)
        speed.append({'name':name,'label':label,'result':result,'peak_command_mps':peak,'endpoint_passed':result['ground_truth_goal_error']<.35 and result['goal_error']<.35 and result['stopped_speed']<.03})
fig,ax=plt.subplots(figsize=(11,4.4));bars=ax.bar(range(len(speed)),[v['result']['ground_truth_goal_error'] for v in speed],color=['#22926c' if v['endpoint_passed'] else '#ca6046' for v in speed]);ax.axhline(.35,c='black',ls='--',label='Acceptance limit 0.35 m')
for bar,value in zip(bars,speed):ax.text(bar.get_x()+bar.get_width()/2,bar.get_height()+.015,f'{value["result"]["ground_truth_goal_error"]:.3f}m',ha='center',fontsize=10)
ax.set_xticks(range(len(speed)),[v['label'] for v in speed],fontsize=9);ax.set_ylabel('Stopped endpoint error against Gazebo truth (m)');ax.set_title('Same far goal: original 2.5 m/s cap, one factor changed at a time');ax.set_ylim(0,1.23);ax.legend();ax.grid(axis='y',alpha=.2);fig.tight_layout();fig.savefig(out/'C020-braking-comparison.png',dpi=150);plt.close(fig)

full=[];fig,ax=plt.subplots(figsize=(9,3.7))
for name in ['early-brake-full-01','early-brake-full-02']:
    path=root/name/'audit/suite-summary.json'
    if not path.exists():continue
    value=json.loads(path.read_text());motion=[]
    for case in value['cases']:
        result=next((m for m in case.get('metrics',[]) if m.get('event')=='result'),None)
        if result:motion.append({'case':case['name'],'result':result})
    full.append({'name':name,'completed':value['completed'],'motion':motion,'backup_events':value.get('backup_events_in_interval'),'empty_plan_events':value.get('empty_plan_events_in_interval')})
    ax.plot(range(len(motion)),[v['result']['ground_truth_goal_error'] for v in motion],marker='o',label=name)
ax.axhline(.35,c='black',ls='--');ax.set_xticks(range(6),['Near','Far','Center','SW','NW','Far repeat']);ax.set_ylabel('Stopped truth error (m)');ax.set_ylim(0,.4);ax.set_title('2.5 m/s cap + 2m approach: fresh-start full routes');ax.legend();ax.grid(alpha=.2);fig.tight_layout();fig.savefig(out/'C020-full-routes.png',dpi=150);plt.close(fig)

obstacles=[];fig,axes=plt.subplots(3,2,figsize=(11,10.8))
for ax,name in zip(axes.flat,['obstacle-control-02','obstacle-parked-02','early-brake-obstacle-control','early-brake-obstacle-parked','early-brake-obstacle-diagnostics','early-brake-obstacle-confirm']):
    case=root/name;path=case/'session.json'
    if not path.exists():ax.set_visible(False);continue
    meta=json.loads(path.read_text());samples=[v for v in rows(case/'samples.jsonl') if v.get('phase')=='goal'];goal=metrics(case/'goal')
    if samples:
        xy=np.array([v['truth_map'][:2] for v in samples]);stamp=np.array([v['sim'] for v in samples]);cuts=np.r_[0,np.where(np.diff(stamp)>.3)[0]+1,len(stamp)]
        for a,z in zip(cuts[:-1],cuts[1:]):ax.plot(xy[a:z,0],xy[a:z,1],c='#c42f91',lw=2,label='Actual robot route' if a==0 else None)
        ax.plot([1,5],[-5.5,-5.5],':',c='gray');ax.scatter([1,5],[-5.5,-5.5],c=['green','red'],s=80,marker='*')
        for snapshot in ['cart-detected','before-cart']:
            p=case/(snapshot+'-costmaps.json')
            if p.exists():
                grid=json.loads(p.read_text()).get('global_costmap');data=np.array(grid['data']);ox,oy=grid['origin'];res=grid['resolution'];ax.imshow(data,origin='lower',extent=[ox,ox+data.shape[1]*res,oy,oy+data.shape[0]*res],vmin=0,vmax=100,cmap='Greys',alpha=.35);break
        if meta.get('scenario')=='parked':
            ax.add_patch(Rectangle((2.71,-5.75),.58,.5,facecolor='#1679b5',edgecolor='black',alpha=.75,label='Temporary obstacle'))
            active=[v for v in samples if v.get('cart_map')]
            if active:
                v=min(active,key=lambda v:v.get('footprint_clearance_m',1000));x,y,a=v['truth_map'];c,s=math.cos(a),math.sin(a);points=[[x+c*dx-s*dy,y+s*dx+c*dy] for dx,dy in [(-.29,-.25),(.29,-.25),(.29,.25),(-.29,.25)]];ax.add_patch(Polygon(points,fill=False,edgecolor='red',lw=1.5))
    error=next((v['ground_truth_goal_error'] for v in goal if v.get('event')=='result'),None);gap=meta.get('min_footprint_clearance_m');detail=f'error {error:.3f}m' if error is not None else 'no goal result'
    if gap is not None:detail+=f', clearance {gap:.3f}m'
    ax.set_title(name+'\n'+meta['status']+': '+detail,fontsize=10);ax.set_xlim(.4,5.6);ax.set_ylim(-6.9,-4.1);ax.set_aspect('equal',adjustable='box');ax.set_xlabel('map x (m)');ax.set_ylabel('map y (m)');ax.legend(fontsize=8,loc='upper left')
    obstacles.append({'name':name,'summary':meta,'goal_metrics':goal})
fig.tight_layout();fig.savefig(out/'C020-static-obstacles.png',dpi=150);plt.close(fig)
(out/'C020-summary.json').write_text(json.dumps({'speed_cases':speed,'full_routes':full,'static_obstacles':obstacles,'scope':'Static obstacle avoidance only; historical crossing records excluded from current scope.'},ensure_ascii=False,indent=2))
print(json.dumps({'output_dir':str(out),'speed_cases':len(speed),'full_routes':len(full),'static_obstacles':len(obstacles)}))
