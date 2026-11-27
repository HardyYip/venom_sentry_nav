"""Produce a navigation-only Gazebo test cart from the ODIN candidate and archived fixtures."""
import copy
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml

destination = Path(sys.argv[1])
case_path = Path(sys.argv[2])
base = Path('/baseline')
case = json.loads((case_path / 'session.json').read_text())
model_file = base / 'fixed-left/cases/left-near-01/stack-evidence/robot-red_standard_robot1.sdf'
archived = ET.parse(model_file)
candidate = ET.parse('/trial/odin-up15.sdf')
start = case['start']
goal = [b-a for a,b in zip(start, case['goal'])]
heading = math.atan2(goal[1], goal[0])
root = ET.Element('sdf', version='1.8')
world = ET.SubElement(root,'world',name='navigation')
def tag(parent, name, value):
    e=ET.SubElement(parent,name);e.text=str(value);return e
def plugin(parent, name, library, **fields):
    element=ET.SubElement(parent,'plugin',filename='libignition-gazebo-'+library+'-system.so',name='ignition::gazebo::systems::'+name)
    for k,v in fields.items():tag(element,k,v)
    return element
def shape(link, kind, dims, pose, name):
    s=ET.SubElement(link,kind,name=name);tag(s,'pose',pose)
    g=ET.SubElement(s,'geometry');tag(ET.SubElement(g,'box'),'size',' '.join(map(str,dims)))
def make_box(name,x,y,dims,static):
    m=ET.SubElement(world,'model',name=name);tag(m,'static',str(static).lower());tag(m,'pose',f'{x} {y} 0 0 0 0')
    link=ET.SubElement(m,'link',name='body');tag(link,'gravity','false')
    for kind in ('visual','collision'):
        shape(link,kind,dims,f'0 0 {dims[2]/2} 0 0 0','body_'+kind)
    return m,link
tag(world,'gravity','0 0 0')
physics=ET.SubElement(world,'physics',name='fixed_step',type='ignored')
tag(physics,'max_step_size',0.002);tag(physics,'real_time_factor',1)
for name,library in [('Physics','physics'),('UserCommands','user-commands'),('SceneBroadcaster','scene-broadcaster'),('Contact','contact')]:
    plugin(world,name,library)
plugin(world,'Sensors','sensors',render_engine='ogre2')
chassis,link=make_box('cart',0,0,[.58,.53,.12],False)
for geometry in list(link.findall('visual'))+list(link.findall('collision')):
    geometry.find('pose').text='0 0 .115 0 0 0'
touch=ET.SubElement(link,'sensor',name='contacts',type='contact')
tag(touch,'always_on','true');tag(touch,'update_rate',50);tag(touch,'topic','/cart/contact')
tag(ET.SubElement(touch,'contact'),'collision','body_collision')
plugin(chassis,'VelocityControl','velocity-control',topic='/cart/cmd')
plugin(chassis,'OdometryPublisher','odometry-publisher',odom_topic='/cart/odom',odom_frame='odom',robot_base_frame='base_link',odom_publish_frequency=50,dimensions=3)
pan=heading+math.radians(80)
def rotate_xy(x,y):
    return (math.cos(pan)*x-math.sin(pan)*y,math.sin(pan)*x+math.cos(pan)*y)
poses={}
for key in ('odin','mid'):
    if key=='odin':
        source=candidate.find('.//sensor')
        x,y,z,r,p,yaw=map(float,source.findtext('pose').split())
    else:
        source=archived.find(".//sensor[@name='front_mid360_lidar']")
        j=next(j for j in archived.iter('joint') if j.findtext('child')=='front_mid360')
        x,y,z,r,p,yaw=map(float,j.findtext('pose').split())
        offset=float(source.findtext('pose').split()[2]);x+=offset*math.sin(p)*math.cos(yaw);y+=offset*math.sin(p)*math.sin(yaw);z+=offset*math.cos(p)
    x,y=rotate_xy(x,y)
    pose=[x,y,z+.2136,r,p,yaw+pan]
    s=copy.deepcopy(source);s.find('pose').text=' '.join(map(str,pose))
    for k,v in [('topic','/'+key+'/scan'),('ignition_frame_id',key+'_scan')]:
        el=s.find(k)
        if el is None: el=ET.SubElement(s,k)
        el.text=v
    link.append(s);poses[key]=pose
field=ET.parse(base/'instrumentation/field.sdf').find('model')
for uri in field.iter('uri'):
    uri.text=uri.text.replace('model://','file:///field-models/')
tag(field,'pose',f'{4.8-start[0]} {-3.-start[1]} 0 0 0 0')
world.append(field)
moving='opponent_move_command' in case
if moving:
    trajectory=case['opponent_move_command']
    positions=[trajectory['start_map']]
    dims=[.58,.5,.5]
else:
    positions=case['boxes'];dims=case['box_size']
for i,(x,y) in enumerate(positions):
    m,_=make_box('box'+str(i),x-start[0],y-start[1],dims,not moving)
    if moving:
        plugin(m,'VelocityControl','velocity-control',topic='/box/cmd')
        plugin(m,'OdometryPublisher','odometry-publisher',odom_topic='/box/odom',odom_publish_frequency=30)
ET.ElementTree(root).write(destination/'world.sdf',encoding='utf-8',xml_declaration=True)
(destination/'case.json').write_text(json.dumps(dict(name=case['case'],start=start,goal=goal,boxes=positions,dimensions=dims,moving=moving,poses=poses,trajectory=case.get('opponent_move_command'))))
cfg=yaml.safe_load((base/'fixed-left/cases/left-near-01/stack-evidence/nav2_params.yaml').read_text())
for name in ('global_costmap','local_costmap'):
    par=cfg[name][name]['ros__parameters'];par['robot_base_frame']='base_link'
    for layer in ('odin_obstacle_layer','mid_geometric_obstacle_layer','intensity_voxel_layer'):
        block=par[layer]
        for src in block['observation_sources'].split():
            block[src]['topic']=block[src]['topic'].replace('<robot_namespace>','')
    par['use_sim_time']=True
for name in ('planner_server','controller_server','velocity_smoother','behavior_server','bt_navigator'):
    cfg[name]['ros__parameters']['use_sim_time']=True
for name in ('bt_navigator','behavior_server'):
    cfg[name]['ros__parameters']['robot_base_frame']='base_link'
bt=cfg['bt_navigator']['ros__parameters']
bt['default_nav_to_pose_bt_xml']='/baseline/planner-install/rmul_c072_planner/share/rmul_c072_planner/behavior_trees/navigate_field_refresh.xml'
bt.pop('default_nav_through_poses_bt_xml',None)
for name in ('controller_server','velocity_smoother','bt_navigator'):
    cfg[name]['ros__parameters']['odom_topic']='/cart/odom'
(destination/'nav.yaml').write_text(yaml.safe_dump({k:cfg[k] for k in ('global_costmap','local_costmap','planner_server','controller_server','velocity_smoother','bt_navigator','behavior_server')}))
