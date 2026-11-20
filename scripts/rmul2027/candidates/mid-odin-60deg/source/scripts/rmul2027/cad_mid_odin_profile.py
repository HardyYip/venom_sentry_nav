"""Opt-in MID + Odin1 simulation candidate; no physical calibration claim."""
import argparse, hashlib, json, math, os, copy
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from cad_dual_mid360_profile import prepare as dual_prepare
from dual_mid360_profile import replace_file, linked_tree
from cad_robot_body import load_manifest

HERE=Path(__file__).resolve().parent
DEFAULT_BODY=Path('/mnt/c/Users/Hardy/Documents/Codex/2026-10-18/headoff-md1/outputs/cad-single-mid-C074-artifacts/cad-body-03/body-manifest-slabs.json')

def prepare(output, source, body_manifest=DEFAULT_BODY):
    from ament_index_python.packages import get_package_share_directory
    costmap_navigation = os.environ.get('RMUL_MID_ODIN_C072_COSTMAP') == '1'
    initial_view = os.environ.get('RMUL_MID_ODIN_INITIAL_VIEW') == '1'
    prepare_goal = os.environ.get('RMUL_MID_ODIN_C072_FULL') == '1'
    if prepare_goal and (not initial_view or os.environ.get('RMUL_MID_ODIN_VALID_PATH') == '1'):
        raise ValueError('C072 full transfer requires startup observation and the original periodic replanning')
    if initial_view and not costmap_navigation:
        raise ValueError('Initial direction observation requires C072 costmap navigation')
    if costmap_navigation and (os.environ.get('RMUL_MID_ODIN_VIEW') != '1' or
                               os.environ.get('RMUL_MID_GEOMETRIC_GUARD') != '1'):
        raise ValueError('C072 costmap navigation requires both obstacle branches and the existing stop heartbeat')
    d=dual_prepare(output,source)
    body_manifest,body=load_manifest(body_manifest)
    desc=Path(d['prefix'])/'share/pb2025_robot_description'
    sim=Path(d['prefix'])/'share/rmu_gazebo_simulator'
    template=desc/'resource/xmacro/simulation_robot.sdf.xmacro'
    root=ET.fromstring(template.read_text())
    block=root.find(".//xmacro_block[@name='aux_livox']")
    old=d['mounts']['auxiliary']
    # Candidate optical origin at the old CAD cover centre; NOT real calibration.
    xyz=list(old['cover_center_base_m']); xyz[2]-=.2136
    yaw=math.atan2(old['rotation_base_sensor'][1][2],old['rotation_base_sensor'][0][2])
    pitch=math.radians(10)
    block.set('name','odin1');block.set('prefix','odin_')
    block.set('pose',' '.join(str(v) for v in xyz+[0,pitch,yaw]))
    block.set('update_rate','10');block.set('samples','240')
    inc=next(x for x in root.findall('xmacro_include') if 'aux_mid360' in x.get('uri',''))
    inc.set('uri','model://odin1/model.sdf.xmacro')
    replace_file(template,ET.tostring(root,encoding='unicode'))
    native=ET.parse(desc/'resource/models/aux_mid360/model.sdf.xmacro').getroot()
    b=native.find(".//xmacro_define_block[@name='aux_livox']")
    b.set('name','odin1')
    link=b.find('link');link.set('name','odin_link')
    joint=b.find('joint');joint.set('name','odin_mount_joint');joint.find('child').text='odin_link'
    link.find('pose').set('relative_to','odin_mount_joint')
    for element in list(link):
        if element.tag in ['visual','collision','sensor']:link.remove(element)
    # Body dimensions from vendor, front optical plane x=0, no detailed CAD claim.
    for kind in ['visual','collision']:
        e=ET.SubElement(link,kind,name='odin1_body_'+kind)
        ET.SubElement(e,'pose').text='-0.0215 0 0 0 0 0'
        ET.SubElement(ET.SubElement(ET.SubElement(e,'geometry'),'box'),'size').text='0.043 0.100 0.062'
        if kind=='visual':
            mat=ET.SubElement(e,'material')
            ET.SubElement(mat,'ambient').text='0.08 0.10 0.13 1'
            ET.SubElement(mat,'diffuse').text='0.12 0.15 0.18 1'
    window=ET.SubElement(link,'visual',name='odin1_optical_window')
    ET.SubElement(window,'pose').text='0.0005 0 0 0 0 0'
    ET.SubElement(ET.SubElement(ET.SubElement(window,'geometry'),'box'),'size').text='0.001 0.085 0.045'
    mat=ET.SubElement(window,'material')
    ET.SubElement(mat,'ambient').text='0.05 0.4 0.65 1'
    ET.SubElement(mat,'diffuse').text='0.05 0.4 0.65 1'
    sensor=ET.SubElement(link,'sensor',name='odin_lidar',type='gpu_lidar')
    for tag,value in [('ignition_frame_id','odin_scan'),('pose','0 0 0 0 0 0'),('always_on','true'),('visualize','false'),('update_rate','10')]:
        ET.SubElement(sensor,tag).text=value
    ray=ET.SubElement(sensor,'ray');scan=ET.SubElement(ray,'scan')
    for axis,samples,half in [('horizontal',240,math.pi/3),('vertical',180,math.pi/4)]:
        a=ET.SubElement(scan,axis)
        for key,value in [('samples',samples),('resolution',1),('min_angle',-half),('max_angle',half)]:
            ET.SubElement(a,key).text=str(value)
    rg=ET.SubElement(ray,'range')
    for key,value in [('min',.2),('max',30.),('resolution',.01)]:ET.SubElement(rg,key).text=str(value)
    noise=ET.SubElement(sensor,'noise')
    for key,value in [('type','gaussian'),('mean','0'),('stddev','0.03')]:ET.SubElement(noise,key).text=value
    # The block is instantiated by the main robot, no standalone MID test model.
    for element in list(native):
        if element is not b:native.remove(element)
    odir=desc/'resource/models/odin1';odir.mkdir()
    omodel=odir/'model.sdf.xmacro';omodel.write_text(ET.tostring(native,encoding='unicode'))
    bridge=sim/'config/ros_gz_bridge.yaml';values=yaml.safe_load(bridge.read_text())
    aux=next(v for v in values if 'auxiliary/livox/lidar' in v['ros_topic_name'])
    aux.update(ros_topic_name='/<robot_name>/odin/points',
        gz_topic_name='/world/default/model/<robot_name>/link/odin_link/sensor/odin_lidar/scan/points')
    replace_file(bridge,yaml.safe_dump(values,sort_keys=False))
    frames=[d['scan_frames'][0],dict(mount='odin_link',frame='odin_scan',xyz=[0.,0.,0.])]
    launch=sim/'launch/spawn_robots.launch.py';text=launch.read_text()
    old_call='add_scan_frames(robot_urdf_xml, '+repr(d['scan_frames'])+')'
    assert text.count(old_call)==1
    text=text.replace(old_call,'add_scan_frames(robot_urdf_xml, '+repr(frames)+')')
    for name in ['cad_robot_body.py','cad_mid_odin_body.py']:
        replace_file(sim/'launch'/name,(HERE/name).read_text())
    text=text.replace('from cad_scan_frames import add_scan_frames','from cad_scan_frames import add_scan_frames\nfrom cad_mid_odin_body import apply_body, restore_file_meshes')
    anchor='        robot_xml = xmacro.to_string()';assert text.count(anchor)==1
    text=text.replace(anchor,anchor+'\n        robot_xml = apply_body(robot_xml, '+repr(str(body_manifest))+')')
    anchor='        robot_urdf_xml = urdf_generator.to_string()';assert text.count(anchor)==1
    text=text.replace(anchor,anchor+'\n        robot_urdf_xml = restore_file_meshes(robot_urdf_xml,robot_xml)')
    anchor='"-string",\n                robot_xml,';assert text.count(anchor)==1
    text=text.replace(anchor,'"-file",\n                str(Path(os.environ["ROS_LOG_DIR"]).parent/f"robot-{robot[\'name\']}.sdf"),')
    compile(text,str(launch),'exec');replace_file(launch,text)
    params=output/'nav2-dual-mid.yaml';cfg=yaml.safe_load(params.read_text())
    converter=cfg.pop('aux_mid_obstacle_cloud')
    converter['ros__parameters']['source_frame']='odin_scan'
    cparams=output/'odin-obstacle-cloud.yaml'
    cparams.write_text(yaml.safe_dump({'/red_standard_robot1/odin_obstacle_cloud':converter}))
    for which in ['local','global']:
        layer=cfg[which+'_costmap'][which+'_costmap']['ros__parameters']
        obstacle=layer.pop('aux_mid_obstacle_layer')
        src=obstacle.pop('aux_mid_cloud');src.update(topic='<robot_namespace>/odin/obstacles',sensor_frame='odin_scan')
        obstacle['observation_sources']='odin_cloud';obstacle['odin_cloud']=src
        layer['odin_obstacle_layer']=obstacle
        layer['plugins']=['odin_obstacle_layer' if x=='aux_mid_obstacle_layer' else x for x in layer['plugins']]
    geometric_guard=os.environ.get('RMUL_MID_GEOMETRIC_GUARD')=='1'
    if geometric_guard:
        if os.environ.get('RMUL_MID_ODIN_VIEW')!='1':raise ValueError('MID geometry requires original view safety gate')
        footprint=json.loads(cfg['global_costmap']['global_costmap']['ros__parameters']['footprint'])
        self_radius=max(math.hypot(x,y)for x,y in footprint)
        assert .3<self_radius<.5-.05/math.sqrt(2),'CAD self mask must remain inside original hard reserve'
        geo={'source_frame':'front_mid360_scan','filter_frame':'base_footprint','self_radius':self_radius,'min_height':.06,'max_height':2.,'min_range':.2,'max_range':5.,'voxel_size':.025}
        gparams=output/'mid-geometric-obstacle-cloud.yaml'
        gparams.write_text(yaml.safe_dump({'/red_standard_robot1/mid_geometric_obstacle_cloud':{'ros__parameters':geo}}))
        for which in ['local','global']:
            layers=cfg[which+'_costmap'][which+'_costmap']['ros__parameters']
            extra=copy.deepcopy(layers['odin_obstacle_layer'])
            geo_input=extra.pop('odin_cloud');geo_input.update(topic='<robot_namespace>/mid/geometric_obstacles',sensor_frame='front_mid360_scan')
            extra['observation_sources']='mid_geometric_cloud';extra['mid_geometric_cloud']=geo_input
            layers['mid_geometric_obstacle_layer']=extra
            layers['plugins'].insert(layers['plugins'].index('inflation_layer'),'mid_geometric_obstacle_layer')
        d['mid_geometric_obstacle_branch']={'source':'same raw MID as localization, separate source-stamped obstacle output','self_mask_radius_m':self_radius,'existing_mid_localization_and_terrain_unchanged':True,'existing_odin_unchanged':True,'view_requires_matching_fresh_output':True,'physical_qualification':'NOT_RUN'}
        d['files'][str(gparams)]=''
        d['files'][str(HERE/'mid_geometric_obstacle_cloud.py')]=''
        d['files'][str(HERE/'source_time_obstacle_queue.py')]=''
    cfg['slam_toolbox']['ros__parameters']['scan_topic']='obstacle_scan'
    assert cfg['controller_server']['ros__parameters']['FollowPath']['max_speed_xy']==.5
    if prepare_goal:
        controller = cfg['controller_server']['ros__parameters']['FollowPath']
        if controller['BaseObstacle.scale'] != .4 or controller['planning_clearance_radius'] != .5:
            raise ValueError('C072 full transfer requires the original low-speed controller parameters')
        cfg['planner_server']['ros__parameters']['GridBased']['plugin'] = 'rmul_c072_planner/ReserveNavfn'
        if os.environ.get('RMUL_MID_ODIN_C072_LOCAL_PREDICTION')=='1':
            controller['critics'].insert(0,'rmul_c072_planner::PredictedObstacle')
            controller['rmul_c072_planner::PredictedObstacle.scale']=120.
            d['observed_motion_local_scoring']=True
        if os.environ.get('RMUL_MID_ODIN_C072_SWEPT_CLEARANCE')=='1':
            controller['critics'].append('rmul_c072_planner::SweptClearance')
            controller.setdefault('rmul_c072_planner::SweptClearance.scale',160.)
            d['swept_chassis_local_scoring']=True
    if os.environ.get('RMUL_MID_ODIN_VALID_PATH')=='1':
        if os.environ.get('RMUL_MID_ODIN_VIEW')!='1':
            raise ValueError('Valid-path candidate requires the existing view safety gate')
        tree=HERE/'behavior_trees/navigate_mid_odin_valid_path.xml'
        cfg['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(tree)
        d['files'][str(tree)]=''
        d['global_path_policy']='same goal and currently valid path retained; original 3Hz validation and all recovery/safety limits'
    params.write_text(yaml.safe_dump(cfg,sort_keys=False))
    bringup=Path(d['prefix'])/'share/pb2025_nav_bringup'
    linked_tree(Path(get_package_share_directory('pb2025_nav_bringup')),bringup)
    (Path(d['prefix'])/'share/ament_index/resource_index/packages/pb2025_nav_bringup').write_text('')
    if prepare_goal:
        components=bringup/'launch/navigation_launch.py'
        source_text=components.read_text()
        composed='                name="bt_navigator",\n                parameters=[configured_params],'
        standalone='                name="bt_navigator",\n                output="screen",'
        channels=('send_goal','get_result','cancel_goal','feedback','status')
        remaps=[('navigate_to_pose/_action/'+channel,'navigate_to_pose_internal/_action/'+channel)
                for channel in channels]
        assert source_text.count(composed)==1 and source_text.count(standalone)==1
        source_text=source_text.replace(composed,composed+'\n                remappings='+repr(remaps)+',')
        left, right=source_text.split(standalone)
        remappings='                remappings=[\n'
        assert remappings in right
        source_text=left+standalone+right.replace(remappings,remappings+
            ''.join('                    '+repr(remap)+',\n' for remap in remaps),1)
        compile(source_text,str(components),'exec')
        replace_file(components,source_text)
        d['files'][str(components)]=''
    nav=bringup/'launch/rm_navigation_simulation_launch.py';text=nav.read_text()
    text=text.replace('from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription','from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess')
    command=['python3',str(HERE/'aux_mid_obstacle_cloud.py'),'--ros-args','-r','__node:=odin_obstacle_cloud','-r','__ns:=/red_standard_robot1','-r','/tf:=tf','-r','/tf_static:=tf_static','-r','auxiliary/livox/lidar:=odin/points','-r','auxiliary/obstacles:=odin/obstacles','--params-file',str(cparams),'-p','use_sim_time:=true']
    if os.environ.get('RMUL_ODIN_BOUNDED_SOURCE_TF')=='1':
        d['files'][str(HERE/'aux_mid_obstacle_cloud.py')]=''
        d['files'][str(HERE/'source_time_obstacle_queue.py')]=''
    anchor='    ld.add_action(bringup_cmd)';assert text.count(anchor)==1
    text=text.replace(anchor,anchor+'\n    ld.add_action(ExecuteProcess(cmd='+repr(command)+', output="screen"))')
    if geometric_guard:
        geo_command=['python3',str(HERE/'mid_geometric_obstacle_cloud.py'),'--ros-args','-r','__ns:=/red_standard_robot1','-r','/tf:=tf','-r','/tf_static:=tf_static','--params-file',str(gparams),'-p','use_sim_time:=true']
        text=text.replace(anchor,anchor+'\n    ld.add_action(ExecuteProcess(cmd='+repr(geo_command)+', output="screen"))')
    if os.environ.get('RMUL_MID_ODIN_VIEW')=='1':
        runtime=HERE/('mid_odin_costmap_gate.py' if costmap_navigation else 'mid_odin_view_runtime.py')
        view_command=['python3',str(runtime),'--ros-args','-r','__ns:=/red_standard_robot1','-r','/tf:=tf','-r','/tf_static:=tf_static']
        if geometric_guard and not costmap_navigation:view_command+=['-p','mid_geometric_guard:=true']
        if initial_view:
            initial_command=['python3', str(HERE/'mid_odin_view_runtime.py'), '--ros-args',
                '-r','__ns:=/red_standard_robot1','-r','__node:=mid_odin_initial_view',
                '-r','/tf:=tf','-r','/tf_static:=tf_static',
                '-r','mid_odin_view/stop:=mid_odin_initial_view/stop',
                '-r','mid_odin_view/status:=mid_odin_initial_view/status','-p','mid_geometric_guard:=true']
            if prepare_goal:
                goal_status=['-r','navigate_to_pose/_action/status:=navigate_to_pose_internal/_action/status']
                view_command+=goal_status
                initial_command+=goal_status
                gateway=HERE/'c072_goal_preparation.py'
                d['files'][str(gateway)]=''
            else:
                view_command+=['-p','require_initial_view:=true']
            text=text.replace(anchor,anchor+'\n    ld.add_action(ExecuteProcess(cmd='+repr(initial_command)+', output="screen"))')
        text=text.replace(anchor,anchor+'\n    ld.add_action(ExecuteProcess(cmd='+repr(view_command)+', output="screen"))')
        assert text.count('    ld.add_action(joy_teleop_cmd)')==1
        text=text.replace('    ld.add_action(joy_teleop_cmd)','    # Headless navigation: joystick remains disabled.')
        cfg['fake_vel_transform']['ros__parameters']['mid_odin_view_stop_timeout']=.3
        params.write_text(yaml.safe_dump(cfg,sort_keys=False))
        d.update(active_observation_strategy='Odin directional view with measured raw coverage and source-matched converter; bounded fail-closed heartbeat',view_stop_timeout_s=.3)
        if costmap_navigation:
            d['active_observation_strategy']='C072 costmap navigation with two source-matched sensor streams and fail-closed heartbeat'
            d['files'][str(runtime)]=''
        if initial_view:
            d['active_observation_strategy'] += '; acquire one fresh directional observation per goal before release'
        if prepare_goal:
            d['active_observation_strategy'] += '; preview and measure before accepting the navigation action'
        if not costmap_navigation or initial_view:
            for helper in ['mid_odin_view_runtime.py','mid_odin_view_policy.py','mid_odin_view_geometry.py','mid_odin_view_coverage.py','mid_odin_view_path.py','mid_odin_view_gaze.py']:
                d['files'][str(HERE/helper)]=''
    compile(text,str(nav),'exec');replace_file(nav,text)
    d.update(profile='cad-mid-odin',active_lidar_count=2,scan_frames=frames,physical_scan_pose_unchanged=False,primary_scan_pose_unchanged=True,native_scan_origin_m={'mid':[0.,0.,.03],'odin':[0.,0.,0.]},
      mounts={'mid':d['mounts']['primary'],'odin':dict(parent='gimbal_yaw',parent_xyz_m=xyz,rpy_rad=[0,pitch,yaw])},
      odin=dict(assumed_model='Odin1',hardware_model_confirmed=False,origin='candidate old cover centre, not measured optical calibration',
        horizontal_degrees=120,vertical_degrees=90,samples=[240,180],hz=10,range_m=[.2,30],
        simulation='uniform angular GPU rays; not vendor temporal/perspective pattern; no material-dependent range model',
        body='dimensioned box and window proxy, not manufacturer CAD',onboard_slam_simulated=False,imu_source_for_navigation='original MID only'),
      cad_body_manifest=str(body_manifest),coarse_robot_unchanged=False,
      wheel_and_suspension_dynamics_reconstructed=False,small_yaw_mechanism_reconstructed=False,
      auxiliary_health_interlock_implemented=costmap_navigation)
    d['files'].pop(str(desc/'resource/models/aux_mid360/model.sdf.xmacro'),None)
    extra=[omodel,nav,cparams,body_manifest,HERE/'cad_mid_odin_profile.py',HERE/'cad_mid_odin_body.py',HERE/'cad_robot_body.py',sim/'launch/cad_mid_odin_body.py',sim/'launch/cad_robot_body.py']
    for f in extra:d['files'][str(f)]=''
    for name in d['files']:d['files'][name]=hashlib.sha256(Path(name).read_bytes()).hexdigest()
    (output/'profile.json').write_text(json.dumps(d,ensure_ascii=False,indent=2))
    return d

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output-root',type=Path,required=True);p.add_argument('--params-file',type=Path,required=True);p.add_argument('--body-manifest',type=Path,default=DEFAULT_BODY);a=p.parse_args()
    print(json.dumps(prepare(a.output_root.resolve(),a.params_file.resolve(),a.body_manifest)))
