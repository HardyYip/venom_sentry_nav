import argparse
import json
import sqlite3
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

parser=argparse.ArgumentParser()
parser.add_argument('--record-dir',type=Path,required=True)
args=parser.parse_args()
record=args.record_dir.resolve()
timing=[json.loads(line) for line in (record/'timing.jsonl').read_text().splitlines() if line]
connection=sqlite3.connect('file:'+str(next((record/'inputs').glob('*.db3')))+'?mode=ro',uri=True)
topics={row[0]:(row[1],get_message(row[2])) for row in connection.execute('select id,name,type from topics')}
imu=[];clouds=[];examples=[]
selected=[key for key,(name,kind) in topics.items() if name.endswith('/livox/imu') or name.endswith('/velodyne_points')]
query='select topic_id,data from messages where topic_id in ('+','.join('?' for _ in selected)+') order by timestamp,id'
for identifier,data in connection.execute(query,selected):
    name,kind=topics[identifier]
    msg=deserialize_message(data,kind)
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    if name.endswith('/livox/imu'):imu.append(stamp)
    else:
        clouds.append(stamp)
        if len(examples)<3:
            field=next(v for v in msg.fields if v.name=='time')
            offsets=np.ndarray((msg.height,msg.width),dtype='<f4',buffer=msg.data,offset=field.offset,strides=(msg.row_step,msg.point_step))
            examples.append({'header_stamp':stamp,'point_count':msg.width*msg.height,'time_field_min':float(offsets.min()),'time_field_max':float(offsets.max())})
connection.close()
imu=np.array(imu)
source_dt=np.diff(imu)
gaps=sorted({tuple(gap) for report in timing for gap in report['imu_gaps']})
verified=[]
for left,right in gaps:
    start=int(np.searchsorted(imu,left+1e-8,side='right'))
    end=int(np.searchsorted(imu,right-1e-8,side='left'))
    if end>start:
        verified.append({'internal_previous_stamp':left,'internal_next_stamp':right,'internal_gap_s':right-left,'external_samples_present_between':end-start,'external_first_missing_stamp':float(imu[start]),'external_last_missing_stamp':float(imu[end-1])})
result={'record_dir':str(record),'external_imu_count':len(imu),'external_imu_max_gap_s':float(source_dt.max()) if len(source_dt) else None,'external_imu_nonpositive_intervals':int((source_dt<=0).sum()),'external_cloud_count':len(clouds),'cloud_time_examples':examples,'internal_timing_reports':len(timing),'internal_imu_max_gap_s':max((v['imu_max_gap_s'] for v in timing),default=None),'internal_gap_events':len(gaps),'verified_internal_missing_sample_intervals':len(verified),'verified_missing_sample_count':sum(v['external_samples_present_between'] for v in verified),'verified_examples':verified[:30],'point_time_processed_span_s':max((v['point_offset_max_s'] for v in timing),default=None),'max_scan_processing_wall_s':max((v['scan_processing_wall_max_s'] for v in timing),default=None),'scope':'message missing from Point-LIO callback sequence while present in external bag; does not alone identify DDS queue as the cause'}
(record/'reception-analysis.json').write_text(json.dumps(result,indent=2)+'\n')
if timing:
    sim=np.array([v['sim_time'] for v in timing]);fig,axes=plt.subplots(2,1,figsize=(10,6),sharex=True)
    axes[0].plot(sim,[v['window_imu_max_gap_s'] for v in timing],'.-',label='Largest gap between Point-LIO IMU callbacks')
    if len(source_dt):axes[0].axhline(float(source_dt.max()),color='green',linestyle='--',label='External recorded IMU maximum gap')
    axes[0].set(ylabel='Message timestamp gap (s)',title='Internal callback timestamps vs independently recorded IMU')
    axes[1].plot(sim,[v['scan_processing_wall_max_s'] for v in timing],'.-',label='Largest scan computation duration')
    axes[1].set(xlabel='Simulation time (s)',ylabel='Wall-clock duration (s)')
    for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=8)
    fig.suptitle(record.name);fig.tight_layout();fig.savefig(record/'timing-input-comparison.png',dpi=160);plt.close(fig)
print(json.dumps({k:v for k,v in result.items() if k!='verified_examples'},indent=2))
