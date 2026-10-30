import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

HERE = Path(__file__).resolve().parent


def linked_tree(source, target):
    target.mkdir(parents=True, exist_ok=True)
    for child in source.iterdir():
        dest = target/child.name
        if child.is_dir():
            linked_tree(child, dest)
        else:
            dest.symlink_to(child.resolve())


def replace_file(path, content):
    if path.is_symlink():
        path.unlink()
    path.write_text(content)


def navigation_params(source):
    params = yaml.safe_load(source.read_text())
    controller = params['controller_server']['ros__parameters']
    controller['FollowPath']['discretize_by_time'] = True
    controller['FollowPath']['time_granularity'] = 1./controller['controller_frequency']
    fake = params['fake_vel_transform']['ros__parameters']
    fake['decision_stop_timeout'] = .3
    libraries = params['bt_navigator']['ros__parameters']['plugin_lib_names']
    params['bt_navigator']['ros__parameters']['plugin_lib_names'] = [
        'rmul_pose_fresh_condition_bt_node' if n == 'pb_pose_fresh_condition_bt_node' else n for n in libraries]
    for name in ('local_costmap', 'global_costmap'):
        settings = params[name][name]['ros__parameters']
        if 'rplidar_obstacle_layer' in settings:
            layer = settings.pop('rplidar_obstacle_layer')
            settings['plugins'] = ['aux_mid_obstacle_layer' if x == 'rplidar_obstacle_layer' else x for x in settings['plugins']]
            layer['observation_sources'] = 'aux_mid_cloud'
            cloud = layer.pop('rplidar_cloud')
            cloud.update(topic='<robot_namespace>/auxiliary/obstacles', sensor_frame='aux_mid360', min_obstacle_height=.06)
            layer['aux_mid_cloud'] = cloud
            settings['aux_mid_obstacle_layer'] = layer
        assert settings['aux_mid_obstacle_layer']['aux_mid_cloud']['observation_persistence'] == 2.
    params.pop('rplidar_obstacle_cloud', None)
    params['slam_toolbox']['ros__parameters']['scan_topic'] = 'auxiliary/unused_scan'
    params['aux_mid_obstacle_cloud'] = dict(ros__parameters=dict(source_frame='aux_mid360',
        filter_frame='base_footprint', min_height=.06, max_height=2., min_range=.2, max_range=5., bins=400))
    if os.environ.get('RMUL_IMU_RECOVERY') == '1':
        fake['recovery_stop_timeout'] = .3
        params['point_lio']['ros__parameters']['max_imu_gap'] = .1
        if os.environ.get('RMUL_RECOVERY_GICP_TOPIC'):
            params['small_gicp_relocalization']['ros__parameters']['input_cloud_topic'] = os.environ['RMUL_RECOVERY_GICP_TOPIC']
        if os.environ.get('RMUL_RECOVERY_IMU_TOPIC'):
            params['point_lio']['ros__parameters']['common']['imu_topic'] = os.environ['RMUL_RECOVERY_IMU_TOPIC']
    return params


def prepare(output, params_source):
    from ament_index_python.packages import get_package_share_directory
    output.mkdir(parents=True, exist_ok=False)
    prefix = output/'prefix'
    for package in ('pb2025_robot_description', 'rmu_gazebo_simulator'):
        original = Path(get_package_share_directory(package))
        linked_tree(original, prefix/'share'/package)
        marker = prefix/'share/ament_index/resource_index/packages'/package
        marker.parent.mkdir(parents=True, exist_ok=True)
        marker.write_text('')
    desc = prefix/'share/pb2025_robot_description'
    sim = prefix/'share/rmu_gazebo_simulator'
    template = desc/'resource/xmacro/simulation_robot.sdf.xmacro'
    root = ET.fromstring(template.read_text())
    for child in list(root):
        if child.tag == 'xmacro_include' and 'rplidar_a2' in child.attrib.get('uri', ''):
            root.remove(child)
    block = root.find(".//xmacro_block[@name='rplidar_a2']")
    assert block is not None
    block.attrib.clear()
    block.attrib.update(name='aux_livox', prefix='aux_', parent='chassis',
                        pose='0.155 0 0.1 0 0 0', update_rate='10', samples='400')
    old_xml = ET.fromstring((desc/'resource/models/rplidar_a2/model.sdf.xmacro').read_text())
    old_link = old_xml.find(".//xmacro_define_block[@name='rplidar_a2']/link")
    aux_root = ET.fromstring((desc/'resource/models/mid360/model.sdf.xmacro').read_text())
    aux_block = aux_root.find(".//xmacro_define_block[@name='livox']")
    aux_block.set('name', 'aux_livox')
    aux_link = aux_block.find('link')
    aux_link.remove(aux_link.find('inertial'))
    aux_link.insert(0, copy.deepcopy(old_link.find('inertial')))
    for item in old_link.findall('collision'):
        aux_link.append(copy.deepcopy(item))
    aux_dir = desc/'resource/models/aux_mid360'
    aux_dir.mkdir()
    (aux_dir/'model.sdf.xmacro').write_text(ET.tostring(aux_root, encoding='unicode'))
    root.insert(0, ET.Element('xmacro_include', uri='model://aux_mid360/model.sdf.xmacro'))
    replace_file(template, ET.tostring(root, encoding='unicode'))
    bridge = sim/'config/ros_gz_bridge.yaml'
    values = yaml.safe_load(bridge.read_text())
    values = [v for v in values if 'rplidar' not in v['ros_topic_name']]
    values.append(dict(ros_topic_name='/<robot_name>/auxiliary/livox/lidar',
        gz_topic_name='/world/default/model/<robot_name>/link/aux_mid360/sensor/aux_mid360_lidar/scan/points',
        ros_type_name='sensor_msgs/msg/PointCloud2', gz_type_name='ignition.msgs.PointCloudPacked', direction='GZ_TO_ROS'))
    replace_file(bridge, yaml.safe_dump(values, sort_keys=False))
    params = output/'nav2-dual-mid.yaml'
    params.write_text(yaml.safe_dump(navigation_params(params_source), sort_keys=False))
    files = [template, aux_dir/'model.sdf.xmacro', bridge, params, HERE/'aux_mid_obstacle_cloud.py', Path(__file__).resolve()]
    manifest = dict(profile='dual-mid360', simulation_only=True, physical_extrinsics_calibrated=False,
        auxiliary_health_interlock_implemented=False, prefix=str(prefix), model_resources=str(desc/'resource/models'),
        source_params=str(params_source), files={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in files})
    (output/'profile.json').write_text(json.dumps(manifest, indent=2))
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--params-file', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(prepare(args.output_root.resolve(), args.params_file.resolve())))
