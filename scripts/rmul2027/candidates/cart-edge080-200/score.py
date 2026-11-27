"""Compare both native Gazebo runs to the previously reported geometric edge-speed metric."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import yaml
from PIL import Image

def load(path):
    return json.loads(path.read_text())

def command_counts(requests):
    below_target = 0
    unchanged_below_target = 0
    increased = 0
    reduced = 0
    for _, _, incoming, outgoing in requests:
        change = outgoing - incoming
        if incoming < 0.8:
            below_target += 1
            unchanged_below_target += abs(change) <= 1e-8
        increased += change > 1e-8
        reduced += change < -1e-8
    return dict(profile_requests=len(requests), below_target_requests=below_target,
                unchanged_below_target=unchanged_below_target,
                increased_requests=increased, reduced_requests=reduced)

def edge_score(case_dir, occupied, res, commands):
    case = load(case_dir/'case.json')
    traj = np.asarray(load(case_dir/'trajectory.json'))
    events = {row['event']:row['time'] for row in map(json.loads,(case_dir/'trace.jsonl').read_text().splitlines())}
    low = events['ready']
    high = events.get('goal_response', traj[-1,0])
    edges = np.unique(np.r_[low,np.arange(low+.01,high,.01),high])
    dt = np.diff(edges)
    mid = .5*(edges[:-1]+edges[1:])
    t = traj[:,0]
    xy = np.array([np.interp(mid,t,traj[:,i]) for i in (1,2)]).T + case['start']
    speeds = np.interp(mid,t,traj[:,3])
    # Hold each smoothed velocity until the next command. The cart starts at rest.
    command_speed = np.zeros_like(mid)
    if len(commands):
        index = np.searchsorted(commands[:,0], mid, side='right') - 1
        valid = index >= 0
        command_speed[valid] = np.linalg.norm(commands[index[valid],1:3], axis=1)
    wall = []
    cache={}
    for pos in xy:
        # At 5 cm grid resolution only cells within 1.1 m can affect this band.
        bin_id=tuple(np.floor(pos).astype(int))
        if bin_id not in cache:
            corner=np.array(bin_id)
            cache[bin_id]=occupied[np.all((occupied>=corner-1.11)&(occupied<=corner+2.11),axis=1)]
        delta = np.abs(cache[bin_id]-pos)
        nearby = delta[(delta[:,0]<1.1)&(delta[:,1]<1.1)]
        wall.append(np.linalg.norm(np.maximum(nearby-res/2,0),axis=1).min(initial=np.inf))
    boxes = np.broadcast_to(np.array(case['boxes'],float),(len(mid),len(case['boxes']),2)).copy()
    if case['moving']:
        move = case['trajectory']
        fraction = np.clip((mid-events['box_motion_start'])/move['duration_s'],0,1)
        boxes[:,0,:] += fraction[:,None]*(np.array(move['end_map'])-move['start_map'])
    def integrate(boxes):
        sep = np.maximum(np.abs(xy[:,None,:]-boxes)-np.asarray(case['dimensions'][:2])/2,0)
        dist = np.minimum(wall,np.linalg.norm(sep,axis=2).min(axis=1))
        band = (np.abs(dist-.9)<=res) & (speeds>=.03)
        return (float(dt[band].sum()), float((dt[band]*speeds[band]).sum()),
                float((dt[band]*command_speed[band]).sum()))
    same_metric = integrate(boxes)
    if case['moving']:
        track = np.asarray(load(case_dir/'box-odom.json'))
        for i in (0,1):
            boxes[:,0,i] = np.interp(mid,track[:,0],track[:,i+1])+case['start'][i]
    measured = integrate(boxes)
    return same_metric,measured

def evaluate(root, cap, round_name='whole-route'):
    run = root/round_name
    retry = root/('startup-retry' if round_name == 'whole-route' else round_name+'-retry')
    status = load(run/'summary.json')
    assert status['requested']==status['finished']==len(status['results'])==15
    assert load(root/'contact-positive/check.json')['passed']
    map_file = Path(__file__).resolve().parent/'map/map-boundary-margin.yaml'
    meta = yaml.safe_load(map_file.read_text())
    grid = np.flipud(np.asarray(Image.open(map_file.parent/meta['image']).convert('L')))
    iy,ix=np.nonzero(((255-grid.astype(float))/255)>meta['occupied_thresh'])
    res=meta['resolution']
    occupied=np.asarray(meta['origin'][:2])+(np.column_stack((ix,iy))+.5)*res
    rows=[]
    for info in status['results']:
        case_dir=run/info['case']
        original_error=info['error']
        retry_dir=retry/info['case']
        if not info['passed'] and info['odom_peak']==0 and (retry_dir/'result.json').exists():
            assert load(retry/'summary.json')['sha256']==status['sha256']
            case_dir=retry_dir
            info=load(case_dir/'result.json')
        c=yaml.safe_load((case_dir/'nav.yaml').read_text())
        ctrl=c['controller_server']['ros__parameters']['FollowPath']
        smooth=c['velocity_smoother']['ros__parameters']
        assert ctrl['max_speed_xy']==smooth['max_speed_xy']==cap
        assert ctrl['max_accel_xy']==smooth['max_accel_xy']==1.75
        assert c['local_costmap']['local_costmap']['ros__parameters']['inflation_layer']['inflation_radius']==.9
        cmd=np.asarray(load(case_dir/'commands.json'))
        delta=float(np.linalg.norm(np.diff(cmd[:,1:],axis=0),axis=1).max()) if len(cmd)>1 else None
        same,measured=edge_score(case_dir,occupied,res,cmd) if (case_dir/'trace.jsonl').read_text().strip() else ((0.,0.,0.),(0.,0.,0.))
        refs=load(case_dir/'edge-requests.json')
        rows.append(dict(case=info['case'],record=str(case_dir.relative_to(root.parent)),first_attempt_error=original_error,goal_passed=info['passed'],reason=info['error'],contacts=info['contacts'],
                         min_gap_m=info['gap'],odom_peak_mps=info['odom_peak'],goal_error_m=info['distance_to_goal'],
                         stopped_s=info['stop_seconds'],max_cmd_step_mps=delta,edge_s=same[0],edge_travel_m=same[1],
                         edge_command_integral_m=same[2],
                         using_box_odom_s=measured[0],using_box_odom_m=measured[1],
                         **command_counts(refs)))
    seconds=sum(r['edge_s'] for r in rows)
    average=sum(r['edge_travel_m'] for r in rows)/seconds
    finite_steps=all(r['max_cmd_step_mps'] is not None and r['max_cmd_step_mps']<=.08750001 for r in rows)
    return dict(cap_mps=cap,trials=len(rows),first_attempts_passed=status['passed'],arrived_and_stopped=sum(r['goal_passed'] for r in rows),
                total_contacts=sum(r['contacts'] for r in rows),time_in_edge_band_s=seconds,
                edge_mean_mps=average,box_odom_check_mps=sum(r['using_box_odom_m'] for r in rows)/sum(r['using_box_odom_s'] for r in rows),
                edge_smoothed_command_mean_mps=sum(r['edge_command_integral_m'] for r in rows)/seconds,
                max_odom_mps=max(r['odom_peak_mps'] for r in rows),min_gap_m=min(r['min_gap_m'] for r in rows),
                command_counts={key:sum(row[key] for row in rows) for key in command_counts([])},
                command_step_check=finite_steps,passed=all(r['goal_passed'] for r in rows) and finite_steps and abs(average-.8)<.02,
                rows=rows)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Recompute the geometric edge-speed score from the Gazebo cart recordings.')
    parser.add_argument('--round', default='full-rerun-1795757044818')
    args = parser.parse_args()
    if not args.round.replace('-', '').isalnum():
        parser.error('invalid round name')
    result = evaluate(Path(__file__).resolve().parent/'native-nav', 2.0, args.round)
    print(json.dumps(result, indent=2))
    sys.exit(0 if result['passed'] else 1)
