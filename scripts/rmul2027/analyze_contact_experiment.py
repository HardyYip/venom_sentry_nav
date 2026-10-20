import argparse
import json
import sqlite3
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
args=parser.parse_args()
root=args.record_dir.resolve()
assert (root/'inputs/metadata.yaml').exists(),'Recorder must finish before analysis'
phases=json.loads((root/'audit/run.json').read_text())['phases']
imu=[]
truth=[]
lio=[]
for file in sorted((root/'inputs').glob('*.db3')):
    with sqlite3.connect(str(file)) as db:
        topics=dict(db.execute('select name,id from topics'))
        for suffix,kind in [('livox/imu',Imu),('chassis_odometry_gt',Odometry),('aft_mapped_to_init',Odometry)]:
            identifier=topics.get('/red_standard_robot1/'+suffix)
            if identifier is None:continue
            for received,data in db.execute('select timestamp,data from messages where topic_id=? order by timestamp',(identifier,)):
                message=deserialize_message(data,kind)
                stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
                if suffix=='livox/imu':
                    acc=message.linear_acceleration
                    omega=message.angular_velocity
                    imu.append([received*1e-9,stamp,acc.x,acc.y,acc.z,np.linalg.norm([omega.x,omega.y,omega.z])])
                elif suffix=='chassis_odometry_gt':
                    pos=message.pose.pose.position
                    v=message.twist.twist.linear
                    truth.append([received*1e-9,stamp,pos.x+4.8,pos.y-3,np.linalg.norm([v.x,v.y])])
                else:
                    pos=message.pose.pose.position
                    lio.append([received*1e-9,stamp,pos.x,pos.y,pos.z])
imu=np.asarray(imu).reshape(-1,6)
truth=np.asarray(truth).reshape(-1,5)
lio=np.asarray(lio).reshape(-1,5)
np.savez_compressed(root/'contact-signals.npz',imu=imu,truth=truth,raw_lio=lio)
results=[]
fig,axes=plt.subplots(max(len(phases),1),2,figsize=(12,3*max(len(phases),1)),squeeze=False)
for row,phase in enumerate(phases):
    start=phase['started_wall_epoch']
    end=phase.get('finished_wall_epoch',start)
    a=imu[(imu[:,0]>=start)&(imu[:,0]<=end)]
    t=truth[(truth[:,0]>=start)&(truth[:,0]<=end)]
    l=lio[(lio[:,0]>=start)&(lio[:,0]<=end)]
    report={'name':phase['name'],'passed':phase.get('passed'),'imu_samples':len(a),'truth_samples':len(t),'lio_samples':len(l)}
    origin=float(a[0,1]) if len(a) else 0.
    if len(a):
        norm=np.linalg.norm(a[:,2:5],axis=1)
        over=np.any(np.abs(a[:,2:5])>=29.7,axis=1)
        report.update(imu_norm_peak_m_s2=float(norm.max()),imu_norm_p99_m_s2=float(np.quantile(norm,.99)),imu_norm_p999_m_s2=float(np.quantile(norm,.999)),imu_axis_ge_29_7_count=int(over.sum()),imu_axis_ge_29_7_fraction=float(over.mean()),imu_header_max_gap_s=float(np.diff(a[:,1]).max()) if len(a)>1 else None,imu_header_nonpositive_count=int(np.sum(np.diff(a[:,1])<=0)))
        axes[row,0].plot(a[:,1]-origin,norm,'.',markersize=1)
    if len(t):
        report.update(truth_speed_peak_m_s=float(t[:,4].max()))
    if len(l)>1:
        dt=np.diff(l[:,1])
        step=np.linalg.norm(np.diff(l[:,2:5],axis=0),axis=1)
        report.update(raw_lio_max_step_m=float(step.max()),raw_lio_max_header_gap_s=float(dt.max()),raw_lio_backward_gt_1us_count=int(np.sum(dt < -1e-6)),raw_lio_large_steps_gt_0_1m=int(np.sum(step>.1)))
        axes[row,1].plot(l[1:,1]-origin,step,'.',markersize=2)
    guard=root/'audit'/phase['name']/'guard-trigger.json'
    if guard.exists():
        value=json.loads(guard.read_text())
        report['localization_guard']=value
        stamp=value['stamp']
        for ax in axes[row]:ax.axvline(stamp-origin,color='red',linestyle='--',label='0.35m guard')
        nearby=a[np.abs(a[:,1]-stamp)<=.1]
        if len(nearby):report['imu_norm_peak_within_0_1simsec_of_guard']=float(np.linalg.norm(nearby[:,2:5],axis=1).max())
    for col,ax in enumerate(axes[row]):
        ax.set_title(phase['name']+(' IMU acceleration' if col==0 else 'raw LIO adjacent pose step'))
        ax.set_xlabel('simulation seconds from phase first IMU')
        ax.set_ylabel('m/s²' if col==0 else 'metres')
        ax.grid(alpha=.2)
    results.append(report)
fig.suptitle(root.name+' | raw LIO is before GICP; peaks alone do not establish causation')
fig.tight_layout(rect=(0,0,1,.97))
fig.savefig(root/'contact-signals.png',dpi=150)
plt.close(fig)
report={'record_dir':str(root),'phases':results,'raw_lio_before_gicp':True,'ground_truth_evaluation_only':True,'truth_pose_scope':'MecanumDrive2 odometry publishes planar pose only; no vertical or tilt conclusion from this bag','warning':'One run per geometry; compare same phase and path exposure. Correlation alone is not proof. No trajectory interpolation or online filtering used here.'}
(root/'contact-analysis.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
