#!/usr/bin/env python3
import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.append('/usr/lib/freecad/lib')
import FreeCAD
import MeshPart
import Part

parser = argparse.ArgumentParser()
parser.add_argument('--step', type=Path, required=True)
parser.add_argument('--scene', type=Path, default=Path(__file__).resolve().parent / 'scene')
args = parser.parse_args()
source = Part.read(str(args.step))
assert source.isValid(), 'Invalid source STEP'
transform = FreeCAD.Matrix()
transform.A11 = .001
transform.A12 = transform.A13 = transform.A21 = transform.A22 = transform.A31 = transform.A33 = 0
transform.A23 = .001
transform.A32 = -.001
shape = source.transformGeometry(transform)
model_dir = args.scene / 'models' / 'rmul_2027_3v3'
for name in ('visual', 'collision'):
    (model_dir / 'meshes' / name).mkdir(parents=True, exist_ok=True)

def bounds(item):
    box = item.BoundBox
    return [box.XMin, box.YMin, box.ZMin, box.XMax, box.YMax, box.ZMax]

sdf = ET.Element('sdf', version='1.7')
model = ET.SubElement(sdf, 'model', name='rmul_2027_3v3')
ET.SubElement(model, 'static').text = 'true'
link = ET.SubElement(model, 'link', name='field')
parts = []
for index, part in enumerate(shape.childShapes()):
    name = f'part_{index:03d}'
    record = {'name': name, 'shape_type': part.ShapeType, 'bbox_m': bounds(part)}
    for kind, linear, angular in [('visual', .0015, .2), ('collision', .005, .35)]:
        mesh = MeshPart.meshFromShape(Shape=part, LinearDeflection=linear,
                                      AngularDeflection=angular, Relative=False)
        destination = model_dir / 'meshes' / kind / f'{name}.stl'
        mesh.write(str(destination))
        record[f'{kind}_triangles'] = mesh.CountFacets
        element = ET.SubElement(link, kind, name=f'{kind}_{name}')
        geometry = ET.SubElement(element, 'geometry')
        mesh_element = ET.SubElement(geometry, 'mesh')
        ET.SubElement(mesh_element, 'uri').text = f'model://rmul_2027_3v3/meshes/{kind}/{name}.stl'
        if kind == 'visual':
            material = ET.SubElement(element, 'material')
            color = '0.36 0.40 0.43 1' if index == 0 else '0.15 0.18 0.20 1'
            ET.SubElement(material, 'ambient').text = color
            ET.SubElement(material, 'diffuse').text = color
        else:
            surface = ET.SubElement(element, 'surface')
            friction = ET.SubElement(surface, 'friction')
            ode = ET.SubElement(friction, 'ode')
            ET.SubElement(ode, 'mu').text = '1'
            ET.SubElement(ode, 'mu2').text = '1'
    parts.append(record)
    print(json.dumps(record), flush=True)
ET.indent(sdf)
ET.ElementTree(sdf).write(model_dir / 'model.sdf', encoding='utf-8', xml_declaration=True)
(model_dir / 'model.config').write_text(
    '<?xml version="1.0"?>\n<model><name>RMUL 2027 3V3 V1.0.0</name>'
    '<version>1.0.0</version><sdf version="1.7">model.sdf</sdf>'
    '<description>Derived from official RoboMaster field CAD. '
    'Coordinates (X,Z,-Y)/1000. Static navigation simulation.</description></model>\n')
worlds = args.scene / 'worlds'
worlds.mkdir(parents=True, exist_ok=True)
world_sdf = ET.Element('sdf', version='1.7')
world = ET.SubElement(world_sdf, 'world', name='default')
ET.SubElement(world, 'gravity').text = '0 0 -9.81'
physics = ET.SubElement(world, 'physics', name='physics', type='ignored')
ET.SubElement(physics, 'max_step_size').text = '0.001'
ET.SubElement(physics, 'real_time_factor').text = '1.0'
for system in ('Physics', 'SceneBroadcaster', 'UserCommands', 'Sensors', 'Imu'):
    suffix = {'SceneBroadcaster': 'scene-broadcaster', 'UserCommands': 'user-commands'}.get(system, system.lower())
    plugin = ET.SubElement(world, 'plugin', filename=f'libignition-gazebo-{suffix}-system.so',
                           name=f'ignition::gazebo::systems::{system}')
    if system == 'Sensors':
        ET.SubElement(plugin, 'render_engine').text = 'ogre2'
light = ET.SubElement(world, 'light', name='sun', type='directional')
for tag, value in [('pose', '0 0 10 0 0 0'), ('diffuse', '.9 .9 .9 1'),
                   ('specular', '.2 .2 .2 1'), ('direction', '0 0 -1'), ('cast_shadows', 'false')]:
    ET.SubElement(light, tag).text = value
include = ET.SubElement(world, 'include')
ET.SubElement(include, 'uri').text = 'model://rmul_2027_3v3'
ET.indent(world_sdf)
ET.ElementTree(world_sdf).write(worlds / 'rmul_2027_3v3_world.sdf', encoding='utf-8', xml_declaration=True)
metadata = {'source_filename': args.step.name,
            'source_sha256': hashlib.sha256(args.step.read_bytes()).hexdigest(),
            'source_units': 'mm', 'cad_to_world': [[.001, 0, 0, 0], [0, 0, .001, 0],
                                                   [0, -.001, 0, 0], [0, 0, 0, 1]],
            'bbox_m': bounds(shape), 'ground_z_m': 0.0, 'interior_size_m': [12, 8],
            'parts': parts, 'collision_method': 'static concave triangle mesh, 5mm tessellation; no convex hull'}
(args.scene / 'conversion.json').write_text(json.dumps(metadata, indent=2) + '\n')
print(json.dumps(metadata), flush=True)
