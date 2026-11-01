"""Optional C072 CAD dual-MID overlay, reusing C070 generation and isolated control."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import yaml

from dual_mid360_profile import prepare as prepare_c070, replace_file
from cad_scan_frames import native_scan_origin

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE/'config/cad_mid360_mounts.json'


def fit_native_cover(mesh_path, config):
    root = ET.parse(mesh_path).getroot()
    ns = {'c':'http://www.collada.org/2005/11/COLLADASchema'}
    geometry = next(g for g in root.findall('.//c:geometry',ns)
                    if g.get('id') == config['native_cover_geometry_id'])
    source = geometry.find('./c:mesh/c:source/c:float_array',ns)
    vertices = np.fromstring(source.text,sep=' ').reshape(-1,3)
    points = vertices[vertices[:,1] > config['native_cover_min_y_mm']]
    if len(points) < 50:
        raise ValueError('insufficient native cover registration points')
    matrix = np.column_stack((2*points,np.ones(len(points))))
    fit = np.linalg.lstsq(matrix,np.sum(points*points,axis=1),rcond=None)[0]
    center = fit[:3]
    radius = float(np.sqrt(fit[3]+center@center))
    residual = float(np.max(np.abs(np.linalg.norm(points-center,axis=1)-radius)))
    if abs(radius-22.) > .2 or residual > .1:
        raise ValueError('native mesh cover no longer agrees with CAD sphere')
    node = next(n for n in root.findall('.//c:visual_scene/c:node',ns)
                if n.find('./c:instance_geometry',ns).get('url') == '#'+config['native_cover_geometry_id'])
    native = np.fromstring(node.find('./c:matrix',ns).text,sep=' ').reshape(4,4)
    visual = np.diag([-1.,-1.,1.])
    center_sensor = visual @ (native[:3,:3] @ center + native[:3,3])
    return center_sensor,dict(points=len(points),center_raw_mm=center.tolist(),
                              center_sensor_m=center_sensor.tolist(),radius_mm=radius,
                              maximum_residual_mm=residual)


def mount_poses(config, native_center, yaw_height):
    if config.get('schema') != 1 or config.get('simulation_only') is not True:
        raise ValueError('expected simulation-only C072 mount schema')
    if config.get('parent_frame') != 'gimbal_yaw':
        raise ValueError('both mounts must use the confirmed big-Yaw proxy')
    if {r['role'] for r in config['radars']} != {'primary','auxiliary'} or len(config['radars']) != 2:
        raise ValueError('expected exactly two distinct MID roles')
    q = np.array(config['cad_to_base_rotation'],dtype=float)
    raw = np.array(config['raw_mesh_to_sensor_rotation'],dtype=float)
    offset = np.array(config['cad_to_base_translation_mm'],dtype=float)*.001
    if not np.allclose(q.T@q,np.eye(3)) or np.linalg.det(q) < .999:
        raise ValueError('invalid CAD/base rotation')
    result = {}
    for radar in config['radars']:
        housing = np.array(radar['cad_housing_rotation'],dtype=float)
        rotation = q @ housing @ raw.T
        if not np.isfinite(rotation).all() or not np.allclose(rotation.T@rotation,np.eye(3),atol=1e-6) or np.linalg.det(rotation) < .999:
            raise ValueError('invalid radar rotation')
        axis = q @ np.array(radar['expected_aperture_axis_cad'],dtype=float)
        if not np.allclose(rotation[:,2],axis,atol=1e-6):
            raise ValueError('housing orientation disagrees with aperture axis')
        cap = q @ np.array(radar['cad_cover_sphere_center_mm'],dtype=float)*.001 + offset
        base_origin = cap - rotation @ native_center
        parent_origin = base_origin - np.array([0,0,yaw_height])
        if not np.isfinite(parent_origin).all():
            raise ValueError('nonfinite mount translation')
        rpy = Rotation.from_matrix(rotation).as_euler('xyz')
        result[radar['role']] = dict(parent=config['parent_frame'],
            parent_xyz_m=parent_origin.tolist(),rpy_rad=rpy.tolist(),
            rotation_base_sensor=rotation.tolist(),base_xyz_m=base_origin.tolist(),
            cover_center_base_m=cap.tolist())
    return result



def adapt_prior_pcd(source, source_manifest, destination):
    """Undo the original CAD map encoding; map geometry and raster stay unchanged."""
    original = source.read_bytes()
    marker = b'DATA binary\n'
    header, separator, body = original.partition(marker)
    if not separator or b'FIELDS x y z' not in header or b'TYPE F F F' not in header:
        raise ValueError('expected original binary xyz float32 PCD')
    points = np.frombuffer(body,dtype='<f4').reshape(-1,3)
    if not np.isfinite(points).all():
        raise ValueError('nonfinite original map')
    data = json.loads(source_manifest.read_text())
    if data['file_sha256'][source.name] != hashlib.sha256(original).hexdigest():
        raise ValueError('original map SHA does not match calibration manifest')
    calibration = data['calibration']
    rotation = Rotation.from_quat(calibration['rotation_xyzw']).as_matrix()
    translation = np.array(calibration['translation'])
    base = points.astype(float) @ rotation.T + translation
    encoded = base.astype('<f4')
    error = float(np.max(np.abs(encoded.astype(float)-base)))
    if error > 1e-5:
        raise ValueError('base-frame map conversion roundtrip error')
    if header.startswith(b'#'):
        header = b'# C072 base/map prior; original lidar calibration undone\n'+header.partition(b'\n')[2]
    destination.write_bytes(header+marker+encoded.tobytes())
    return dict(source=str(source),source_sha256=hashlib.sha256(original).hexdigest(),
                encoding='base/map coordinates; identity import, independent of current yaw',
                points=len(points),maximum_conversion_error_m=error,
                original_calibration=calibration,private_path=str(destination),
                sha256=hashlib.sha256(destination.read_bytes()).hexdigest())


def guarded_occupancy(source_yaml, source_manifest, destination):
    """Anticipate the observed one-cell wall-boundary difference, conservatively."""
    from PIL import Image
    from scipy.ndimage import binary_dilation
    original = yaml.safe_load(source_yaml.read_text())
    source_image = (source_yaml.parent/original['image']).resolve()
    provenance = json.loads(source_manifest.read_text())
    for path in (source_yaml,source_image):
        if provenance['file_sha256'][path.name] != hashlib.sha256(path.read_bytes()).hexdigest():
            raise ValueError('original occupancy map SHA mismatch')
    image = np.asarray(Image.open(source_image))
    if not set(np.unique(image)).issubset({0,254}):
        raise ValueError('expected original binary CAD occupancy, no unknown-cell rewrite')
    occupied = image == 0
    expanded = binary_dilation(occupied,structure=np.ones((3,3),dtype=bool),border_value=1)
    target_image = destination/'map-boundary-margin.pgm'
    target_yaml = destination/'map-boundary-margin.yaml'
    Image.fromarray(np.where(expanded,0,254).astype(np.uint8)).save(target_image)
    config = dict(original)
    config['image'] = target_image.name
    target_yaml.write_text(yaml.safe_dump(config,sort_keys=False))
    return dict(yaml=str(target_yaml),image=str(target_image),cells=1,
                margin_m=float(original['resolution']),added_occupied_cells=int((expanded & ~occupied).sum()),
                source_yaml=str(source_yaml),source_image=str(source_image),
                source_yaml_sha256=hashlib.sha256(source_yaml.read_bytes()).hexdigest(),
                source_image_sha256=hashlib.sha256(source_image.read_bytes()).hexdigest(),
                explanation='conservative planner prior for observed neighboring-cell wall marking; runtime safety circle and perception unchanged')


def configure_projection(params, horizon):
    """Enable only a bounded internal planning seed; source state and gates stay adopted."""
    horizon=float(horizon)
    if not np.isfinite(horizon) or not 0 <= horizon <= .1:
        raise ValueError('invalid planning projection horizon')
    params['controller_server']['ros__parameters']['FollowPath']['pose_projection_horizon']=horizon
    return dict(horizon_s=horizon,internal_seed_only=True,source_stamps_unchanged=True)


def configure_planning_clearance(params, radius):
    """Keep the trajectory kernel and goal potential outside the global footprint."""
    cp=params['global_costmap']['global_costmap']['ros__parameters']
    polygon=np.array(ast.literal_eval(cp['footprint']),dtype=float)
    resolution=float(cp['resolution'])
    padding=float(cp['footprint_padding'])
    radius=float(radius)
    if polygon.ndim!=2 or polygon.shape[0]<3 or polygon.shape[1]!=2 or not np.isfinite(polygon).all() or not np.isfinite([resolution,padding,radius]).all() or resolution<=0 or padding<0:
        raise ValueError('invalid planning clearance geometry')
    padded=polygon+np.sign(polygon)*padding
    footprint_bound=float(np.linalg.norm(padded,axis=1).max())
    local_resolution=float(params['local_costmap']['local_costmap']['ros__parameters']['resolution'])
    if not np.isfinite(local_resolution) or local_resolution<=0:raise ValueError('invalid local field resolution')
    half_diagonal=float(resolution*np.sqrt(2)/2)
    required=max(.382883794+.05,footprint_bound+half_diagonal)
    if radius+1e-12<required:
        raise ValueError('planning clearance cannot shrink the global padded/grid bound')
    params['controller_server']['ros__parameters']['FollowPath']['planning_clearance_radius']=radius
    params['controller_server']['ros__parameters']['FollowPath']['planning_cell_coverage']=True
    return dict(radius_m=radius,physical_hard_floor_m=.382883794+.05,
                global_padded_polygon_bound_m=footprint_bound,grid_half_diagonal_m=half_diagonal,
                required_bound_m=required,shared_trajectory_and_goal_field=True,
                complete_goal_field_cell_coverage=True,goal_field_cell_half_diagonal_m=local_resolution*np.sqrt(2)/2,
                goal_field_cell_cover_radius_m=radius+local_resolution*np.sqrt(2)/2)


def prepare(output, params_source, mount_config=DEFAULT_CONFIG):
    manifest = prepare_c070(output,params_source)
    desc = Path(manifest['prefix'])/'share/pb2025_robot_description'
    template = desc/'resource/xmacro/simulation_robot.sdf.xmacro'
    model = ET.fromstring(template.read_text())
    values = {x.get('name'):x.get('value') for x in model.findall('xmacro_define_value')}
    yaw_height = float(values['chassis_height'])+float(values['gimbal_yaw_height'])
    config = json.loads(mount_config.read_text())
    mesh = desc/'resource/models/mid360/meshes/mid360.dae'
    native_center, registration = fit_native_cover(mesh,config)
    poses = mount_poses(config,native_center,yaw_height)
    native_model=desc/'resource/models/mid360/model.sdf.xmacro'
    aux_model=desc/'resource/models/aux_mid360/model.sdf.xmacro'
    offset=native_scan_origin(native_model)
    np.testing.assert_allclose(offset,[0.,0.,.03],atol=1e-12)
    np.testing.assert_allclose(native_scan_origin(aux_model,require_imu=False),offset,atol=1e-12)
    scan_frames=[dict(mount=mount,frame=mount+'_scan',xyz=offset) for mount in ['front_mid360','aux_mid360']]
    for path in [native_model,aux_model]:
        parsed=ET.parse(path).getroot()
        parsed.find(".//sensor[@type='gpu_lidar']/ignition_frame_id").text='${prefix}mid360_scan'
        replace_file(path,ET.tostring(parsed,encoding='unicode'))
    sim=output/'prefix/share/rmu_gazebo_simulator'
    launch_file=sim/'launch/spawn_robots.launch.py'
    assert launch_file.parent.resolve().is_relative_to(output.resolve())
    launch_text=launch_file.read_text()
    anchor='from sdformat_tools.urdf_generator import UrdfGenerator'
    assert launch_text.count(anchor)==1
    launch_text=launch_text.replace(anchor,anchor+'\nimport sys\nsys.path.insert(0,str(Path(__file__).parent))\nfrom cad_scan_frames import add_scan_frames')
    anchor='        robot_urdf_xml = urdf_generator.to_string()'
    assert launch_text.count(anchor)==1
    launch_text=launch_text.replace(anchor,anchor+'\n        robot_urdf_xml = add_scan_frames(robot_urdf_xml, '+repr(scan_frames)+')')
    replace_file(launch_file,launch_text)
    frame_helper=sim/'launch/cad_scan_frames.py'
    replace_file(frame_helper,(HERE/'cad_scan_frames.py').read_text())

    for role,block_name in [('primary','livox'),('auxiliary','aux_livox')]:
        block = model.find(".//xmacro_block[@name='"+block_name+"']")
        if block is None:
            raise ValueError('missing '+block_name)
        pose = poses[role]
        block.set('parent',pose['parent'])
        block.set('pose',' '.join(format(x,'.16g') for x in pose['parent_xyz_m']+pose['rpy_rad']))
    template.write_text(ET.tostring(model,encoding='unicode'))
    params_path = output/'nav2-dual-mid.yaml'
    params = yaml.safe_load(params_path.read_text())
    lio = params['point_lio']['ros__parameters']['mapping']
    norm = float(np.linalg.norm(lio['gravity_init']))
    gravity = np.array(poses['primary']['rotation_base_sensor']).T @ np.array([0,0,-norm])
    lio['gravity'] = gravity.tolist()
    lio['gravity_init'] = gravity.tolist()
    assert lio['extrinsic_est_en'] is False
    np.testing.assert_allclose(lio['extrinsic_R'],np.eye(3).ravel(),atol=1e-12)
    lio['extrinsic_T']=offset
    params['aux_mid_obstacle_cloud']['ros__parameters']['source_frame']='aux_mid360_scan'
    for which,source in [('local','terrain_map'),('global','terrain_map_ext')]:
        layers=params[which+'_costmap'][which+'_costmap']['ros__parameters']
        layers['intensity_voxel_layer'][source]['sensor_frame']='front_mid360_scan'
        layers['aux_mid_obstacle_layer']['aux_mid_cloud']['sensor_frame']='aux_mid360_scan'

    params['sensor_scan_generation']['ros__parameters']['require_valid_tf'] = True
    policy = configure_projection(params,config['pose_projection_horizon_s'])
    planning = configure_planning_clearance(params,config['planning_clearance_radius_m'])
    # In this pinned loader, base_frame is used solely for the prior-map import
    # transform. Equality with lidar_frame deliberately selects identity; robot
    # odometry and recovery still use their existing robot_base_frame.
    params['small_gicp_relocalization']['ros__parameters']['base_frame'] = 'front_mid360'
    map_root = HERE.parents[3]/'artifacts/rmul_2027_3v3/maps-cad'
    prior = adapt_prior_pcd(map_root/'rmul_2027_3v3.pcd',map_root/'manifest.json',output/'prior-base-frame.pcd')
    occupancy = guarded_occupancy(map_root/'rmul_2027_3v3.yaml',map_root/'manifest.json',output)
    params_path.write_text(yaml.safe_dump(params,sort_keys=False))
    sources = [Path(path) for path in manifest['files']]
    sources += [native_model,launch_file,frame_helper,HERE/'cad_scan_frames.py',mount_config,Path(__file__).resolve(),mesh,map_root/'rmul_2027_3v3.pcd',map_root/'manifest.json',output/'prior-base-frame.pcd',
                map_root/'rmul_2027_3v3.yaml',map_root/'rmul_2027_3v3.pgm',Path(occupancy['yaml']),Path(occupancy['image'])]
    manifest.update(scan_frames=scan_frames,native_scan_origin_m=offset,physical_scan_pose_unchanged=True,profile='cad-dual-mid360',mounts=poses,registration=registration,prior_map_import=prior,occupancy_boundary_guard=occupancy,controller_pose_projection=policy,controller_planning_clearance=planning,
        cad_source_sha256=config['source_step_sha256'],parent_role=config['parent_role'],
        virtual_base_convention=config['base_convention'],coarse_robot_unchanged=True,
        precise_occlusion_reconstructed=False,small_yaw_mechanism_reconstructed=False,
        point_lio_gravity_init=gravity.tolist(),
        files={str(f):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources})
    (output/'profile.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--params-file',type=Path,required=True)
    parser.add_argument('--mount-config',type=Path,default=DEFAULT_CONFIG)
    args = parser.parse_args()
    print(json.dumps(prepare(args.output_root.resolve(),args.params_file.resolve(),args.mount_config.resolve())))
