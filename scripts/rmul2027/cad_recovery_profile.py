"""Explicit CAD recovery parameters; preserve the qualified ordinary launcher."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import yaml
HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]
LEGACY_BUILDER_SHA256='8e12fd8884edfdce156904f9198af33cf4c1a30e7bd8966cbd442bdc3209bf82'
def checked_budget(value):
    if isinstance(value,bool) or not isinstance(value,int) or not 1<=value<=64:
        raise ValueError('registration budget must be an integer in 1..64')
    return value
def build_parameters(source,environ=None,budget=64):
    budget=checked_budget(budget)
    environ=os.environ if environ is None else environ
    params=copy.deepcopy(source)
    controller=params['controller_server']['ros__parameters']
    controller['FollowPath']['discretize_by_time']=True
    controller['FollowPath']['time_granularity']=1.0/controller['controller_frequency']
    for name in ('local_costmap','global_costmap'):
        params[name][name]['ros__parameters']['rplidar_obstacle_layer']['rplidar_cloud']['min_obstacle_height']=0.06
    params['fake_vel_transform']['ros__parameters']['decision_stop_timeout']=0.3
    names=params['bt_navigator']['ros__parameters']['plugin_lib_names']
    if names.count('pb_pose_fresh_condition_bt_node')!=1:
        raise ValueError('expected the protected low-speed source parameters')
    params['bt_navigator']['ros__parameters']['plugin_lib_names']=[
        'rmul_pose_fresh_condition_bt_node' if n=='pb_pose_fresh_condition_bt_node' else n for n in names]
    params['fake_vel_transform']['ros__parameters']['recovery_stop_timeout']=0.3
    params['point_lio']['ros__parameters']['max_imu_gap']=0.1
    if environ.get('RMUL_RECOVERY_GICP_TOPIC'):
        params['small_gicp_relocalization']['ros__parameters']['input_cloud_topic']=environ['RMUL_RECOVERY_GICP_TOPIC']
    if environ.get('RMUL_RECOVERY_IMU_TOPIC'):
        params['point_lio']['ros__parameters']['common']['imu_topic']=environ['RMUL_RECOVERY_IMU_TOPIC']
    params['small_gicp_relocalization']['ros__parameters']['registration_max_iterations']=budget
    # Recovery-only sampling and bounded metric correspondence candidate.
    # Solver/termination criteria and source sampling remain unchanged.
    params['small_gicp_relocalization']['ros__parameters']['global_leaf_size']=0.02
    params['small_gicp_relocalization']['ros__parameters']['deterministic_global_sampling']=True
    params['small_gicp_relocalization']['ros__parameters']['metric_correspondences']=True
    return params
def prepare(source,destination,budget=64):
    launcher=REPO/'rmul2027_decision/tools/start_low_speed.sh'
    if hashlib.sha256(launcher.read_bytes()).hexdigest()!=LEGACY_BUILDER_SHA256:
        raise ValueError('qualified ordinary parameter builder changed; review before recovery')
    value=build_parameters(yaml.safe_load(source.read_text()),budget=budget)
    with destination.open('x') as f:yaml.safe_dump(value,f,sort_keys=False)
    receipt=dict(profile='CAD controlled recovery only',source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        ordinary_builder_sha256=LEGACY_BUILDER_SHA256,registration_max_iterations=budget,
        legacy_default=10,max_allowed_iterations=64,target_leaf_size_m=0.02,deterministic_global_sampling=True,metric_correspondences=True,correspondence_neighbor_cap=8,correspondence_extra_distance_m=.025,legacy_target_leaf_size_m=0.1,source_leaf_size_m=0.05,convergence_criteria_unchanged=True,
        safety_thresholds_unchanged=True,params_sha256=hashlib.sha256(destination.read_bytes()).hexdigest())
    with destination.with_suffix('.profile.json').open('x') as f:json.dump(receipt,f,indent=2)
    return receipt
if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--max-iterations',type=int,default=64)
    args=parser.parse_args();print(json.dumps(prepare(args.source,args.output,args.max_iterations)))
