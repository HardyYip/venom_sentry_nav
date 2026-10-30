"""Extract geometry evidence only; component/cover origins are not runtime TFs."""
import argparse
import json
import math
from pathlib import Path




def build(assembly):
    rows=assembly['records']
    root=next(x for x in rows if len(x['path'])==1)
    housings=[x for x in rows if len(x['path'])==2 and x['name'].startswith('MID-360_4_ASM')]
    covers=[x for x in rows if len(x['path'])==2 and x['name'].startswith('MID-360 v1')]
    wheels=[x for x in rows if x['name']=='全向轮总装']
    if len(housings)!=2 or len(covers)!=2 or len(wheels)!=4:
        raise ValueError('Expected two housings, two covers, four wheel instances')
    wheel_plane=max(x['bbox_mm'][5] for x in wheels)
    up=[0.,0.,-1.]
    unused=list(covers);radars=[]
    for index,housing in enumerate(sorted(housings,key=lambda x:'镜像' in x['name'])):
        center=housing['bbox_center_mm']
        cover=min(unused,key=lambda x:math.dist(x['bbox_center_mm'],center))
        unused.remove(cover)
        if ('镜像' in housing['name'])!=('镜像' in cover['name']):
            raise ValueError('Housing/cover pairing inconsistent')
        sphere=next(x for x in cover['features_world'] if x['type']=='sphere' and abs(x['radius_mm']-22.)<.01)
        cylinders=[x for x in cover['features_world'] if x['type']=='cylinder']
        cylinder=max(cylinders,key=lambda x:x['area_mm2'])
        axis=list(cylinder['axis'])
        outward=[a-b for a,b in zip(sphere['center_mm'],center)]
        if sum(a*b for a,b in zip(axis,outward))<0:axis=[-x for x in axis]
        if abs(math.sqrt(sum(x*x for x in axis))-1)>1e-6:
            raise ValueError('Non-unit optical-cover axis')
        point=list(sphere['center_mm'])
        reference=[x*.001 for x in [-point[0],point[1],wheel_plane-point[2]]]
        combined=[min(housing['bbox_mm'][i],cover['bbox_mm'][i]) for i in range(3)]+[max(housing['bbox_mm'][i],cover['bbox_mm'][i]) for i in range(3,6)]
        radars.append(dict(id='primary' if index==0 else 'auxiliary',
            housing=housing['name'],cover=cover['name'],
            housing_instance=housing['instance'],cover_instance=cover['instance'],
            geometric_cover_sphere_center_cad_mm=point,cover_sphere_radius_mm=sphere['radius_mm'],
            geometric_aperture_axis_cad=axis,
            tilt_from_wheel_reference_up_deg=math.degrees(math.acos(max(-1,min(1,sum(a*b for a,b in zip(axis,up)))))),
            combined_bbox_cad_mm=combined,
            reference_view_xyz_m=reference,
            parent_frame=None,kinematic_relation='unconfirmed_by_user',
            lidar_optical_origin='not_calibrated_from_cover_sphere',
            runtime_pose=None))
    return dict(schema=1,profile_id='cad-layout-C071-unconfirmed',status='GEOMETRY_ONLY_NOT_RUNTIME',
        source_sha256=assembly['source_sha256'],source_bytes=assembly['bytes'],
        source_path=assembly['source'],geometry_unit='mm',
        original_cad_bbox_mm=root['bbox_mm'],original_cad_extent_mm=root['extent_mm'],
        wheel_reference_plane_cad_z_mm=wheel_plane,
        geometry_above_wheel_reference_height_mm=wheel_plane-root['bbox_mm'][2],
        geometry_below_wheel_reference_mm=root['bbox_mm'][5]-wheel_plane,
        reference_view=dict(description='visualization coordinates only; not calibrated robot base',
                            rotation_cad_to_view=[[-1,0,0],[0,1,0],[0,0,-1]],
                            translation_mm=[0,0,wheel_plane],
                            ground='static four-wheel CAD geometry, not loaded suspension/real ground calibration'),
        baseline=dict(commit='d193ac3061946e20f9cdfd5481ea27cb2e87ca2a',
            primary_chassis_xyz_m=[.16,0,.18],primary_chassis_rpy_rad=[math.pi/6,0,math.pi/2],
            auxiliary_chassis_xyz_m=[.155,0,.1],auxiliary_chassis_rpy_rad=[0,0,0]),
        radars=radars,
        runtime_ready=False,real_vehicle_ready=False,
        pending=['main/aux fixed versus big-Yaw moving-parent relation',
                 'calibrated CAD-to-robot-base relation under retained chassis proxy',
                 'virtual scan/IMU origin convention versus geometric cover center',
                 'source-time moving TF/locator compatibility if mount follows a joint'],
        scope='two MID poses and necessary exterior only; chassis/gimbal mechanisms unchanged')


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('assembly',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    value=build(json.loads(args.assembly.read_text()))
    args.output.write_text(json.dumps(value,ensure_ascii=False,indent=2))
    print(json.dumps(dict(status=value['status'],bbox=value['original_cad_extent_mm'],
                         radars=[{k:x[k] for k in ['id','geometric_cover_sphere_center_cad_mm','geometric_aperture_axis_cad','tilt_from_wheel_reference_up_deg']} for x in value['radars']]),ensure_ascii=False))
