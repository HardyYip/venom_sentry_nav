import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

artifact=Path('/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3')
record=artifact/'nav-relocalization'
events=[json.loads(line) for line in (record/'verify.jsonl').read_text().splitlines() if line.startswith('{')]
goal=next(e['xy'] for e in events if e['event']=='goal')
result=next(e for e in events if e['event']=='result')
trace=json.loads((record/'trace.json').read_text())
maps=json.loads((record/'maps.json').read_text())
m=maps['map']
grid=np.array(m['data']).reshape(m['height'],m['width'])
origin=m['origin']; resolution=m['resolution']
extent=[origin[0],origin[0]+grid.shape[1]*resolution,origin[1],origin[1]+grid.shape[0]*resolution]
truth=np.array([[e['value'][0]+4.8,e['value'][1]-3] for e in trace if e['kind']=='ground_truth'])
estimate=np.array([e['value'][:2] for e in trace if e['kind']=='estimated_pose'])
commands=[e for e in trace if e['kind']=='cmd_vel']
fig=plt.figure(figsize=(14,8))
layout=fig.add_gridspec(2,2,height_ratios=[3,1])
cmap=ListedColormap(['#b8bec5','#ffffff','#232936'])
norm=BoundaryNorm([-2,-.5,50,101],cmap.N)
for ax,zoom in [(fig.add_subplot(layout[0,0]),False),(fig.add_subplot(layout[0,1]),True)]:
    ax.imshow(grid,origin='lower',extent=extent,cmap=cmap,norm=norm)
    ax.plot(truth[:,0],truth[:,1],color='#137c68',lw=3,label='Gazebo ground truth')
    if len(estimate):
        ax.plot(estimate[:,0],estimate[:,1],color='#ec7c36',lw=1.5,label='TF localization')
    ax.scatter([goal[0]],[goal[1]],marker='*',s=200,c='#c73552',label='Nav2 goal',zorder=5)
    ax.scatter([truth[0,0]],[truth[0,1]],marker='o',s=50,c='#137c68',zorder=5)
    ax.set(xlabel='Map X (m)',ylabel='Map Y (m)',title='Sensor map and measured trajectory' if not zoom else 'Short navigation: start to goal')
    if zoom:
        ax.set_xlim(-.35,1.25);ax.set_ylim(-.65,.65);ax.legend(loc='lower right',fontsize=9)
    ax.set_aspect('equal')
ax=fig.add_subplot(layout[1,:])
if commands:
    times=np.array([e['wall'] for e in commands])
    values=np.array([e['value'] for e in commands])
    ax.plot(times-times[0],values[:,0],label='cmd_vel X',color='#137c68')
    ax.plot(times-times[0],values[:,1],label='cmd_vel Y',color='#ec7c36')
    ax.plot(times-times[0],values[:,2],label='cmd_vel yaw',color='#8059a4')
ax.set(xlabel='Wall time after first command (s)',ylabel='Command (m/s, rad/s)')
ax.grid(alpha=.2);ax.legend(loc='upper right',ncol=3)
fig.suptitle(f'RMUL 2027 3V3 | small_gicp + ThetaStar + OmniPid | SUCCEEDED\n'
             f'Truth movement {result["ground_truth_moved"]:.3f} m | truth goal error {result["ground_truth_goal_error"]:.3f} m | '
             f'estimated goal error {result["goal_error"]:.3f} m | stopped command {result["stopped_command"]:.3f}',fontsize=12)
fig.tight_layout(rect=[0,0,1,.92])
fig.savefig(artifact/'navigation-result.png',dpi=150)
plt.close(fig)
print(artifact/'navigation-result.png')
