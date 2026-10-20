import hashlib
import json
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

folder = Path(__file__).resolve().parent
scene = folder / 'scene'
original = scene / 'models/rmul_2027_3v3'
output = scene / 'models/rmul_2027_3v3_flat_marking'
source = original / 'meshes/collision/part_000.stl'
expected = 'ad09dab6c06b66ed6d0286d9b359157bcb92ec41ce986ff533cb4772cd9a703f'
data = source.read_bytes()
if hashlib.sha256(data).hexdigest() != expected:
    raise SystemExit('Original collision mesh SHA256 changed; inspect before generating.')
count = struct.unpack_from('<I', data, 80)[0]
if count != 3656 or len(data) != 84 + count * 50:
    raise SystemExit('Unexpected original STL layout.')
kept = []
removed = []
ground = []
for index in range(count):
    block = data[84 + index * 50:84 + (index + 1) * 50]
    values = struct.unpack('<12fH', block)
    vertices = [values[i:i + 3] for i in (3, 6, 9)]
    matches = all(abs(x) <= 1.500001 and abs(y) <= 1.500001 and .006999 <= z <= .010001 for x, y, z in vertices)
    if matches:
        removed.append(index)
    else:
        kept.append(block)
        if all(abs(v[2]) < 1e-7 for v in vertices):
            ground.append(vertices)
if len(removed) != 60 or len(ground) != 266:
    raise SystemExit('Expected exactly 60 marking faces and all 266 ground faces; no output written.')

def covers(point, triangle):
    x, y = point
    a, b, c = triangle
    if abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])) < 1e-12:
        return False
    signs = []
    for a, b in zip(triangle, triangle[1:] + triangle[:1]):
        signs.append((b[0] - a[0]) * (y - a[1]) - (b[1] - a[1]) * (x - a[0]))
    return min(signs) >= -1e-8 or max(signs) <= 1e-8

samples = [(-1.475 + ix * .05, -1.475 + iy * .05) for ix in range(60) for iy in range(60)]
missing = [point for point in samples if not any(covers(point, triangle) for triangle in ground)]
if missing:
    raise SystemExit('Original z=0 floor missing under marking: ' + str(missing[:5]))
mesh = b'RMUL2027 central marking visual-only collision variant'.ljust(80, b' ') + struct.pack('<I', len(kept)) + b''.join(kept)
mesh_path = output / 'meshes/collision/part_000.stl'
mesh_path.parent.mkdir(parents=True, exist_ok=True)
mesh_path.write_bytes(mesh)
model = ET.parse(original / 'model.sdf')
model.getroot().find('model').set('name', 'rmul_2027_3v3_flat_marking')
model.getroot().find("model/link/collision[@name='collision_part_000']/geometry/mesh/uri").text = 'model://rmul_2027_3v3_flat_marking/meshes/collision/part_000.stl'
ET.indent(model)
model.write(output / 'model.sdf', encoding='utf-8', xml_declaration=True)
(output / 'model.config').write_text('<?xml version="1.0"?><model><name>RMUL2027 central marking without collision</name><version>1.0.0</version><sdf version="1.7">model.sdf</sdf><description>Central 3m square marking treated as visual only. User-approved simulation assumption; original CAD is retained.</description></model>\n')
world = ET.parse(scene / 'worlds/rmul_2027_3v3_world.sdf')
world.getroot().find('world/include/uri').text = 'model://rmul_2027_3v3_flat_marking'
ET.indent(world)
world.write(scene / 'worlds/rmul_2027_3v3_flat_marking_world.sdf', encoding='utf-8', xml_declaration=True)
manifest = {
    'purpose': 'User-approved central marking treated as visual-only; not official confirmation of physical construction',
    'source_mesh': str(source.relative_to(folder)), 'source_sha256': expected,
    'collision_mesh': str(mesh_path.relative_to(folder)), 'collision_sha256': hashlib.sha256(mesh).hexdigest(),
    'source_triangles': count, 'removed_triangles': len(removed), 'retained_triangles': len(kept),
    'removed_triangle_indices': removed, 'selection_xy_m': [-1.5, -1.5, 1.5, 1.5],
    'selection_z_m': [.007, .010], 'selection_tolerance_m': .000001,
    'ground_z0_triangles_retained': len(ground), 'floor_samples_checked': len(samples),
    'floor_sample_spacing_m': .05, 'floor_missing_samples': len(missing),
    'retained_triangle_records_byte_identical': True, 'visual_meshes_unchanged': True,
    'other_collision_meshes_unchanged': True, 'navigation_maps_unchanged': True,
    'world': 'rmul_2027_3v3_flat_marking_world.sdf', 'original_world': 'rmul_2027_3v3_world.sdf'
}
(scene / 'flat-marking-contact-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print(json.dumps({k:v for k,v in manifest.items() if k != 'removed_triangle_indices'}, indent=2))
