import hashlib
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

folder = Path(__file__).resolve().parent
scene = folder / 'scene'
source = scene / 'models/rmul_2027_3v3/meshes/collision/part_000.stl'
data = source.read_bytes()
expected = 'ad09dab6c06b66ed6d0286d9b359157bcb92ec41ce986ff533cb4772cd9a703f'
if hashlib.sha256(data).hexdigest() != expected:
    raise SystemExit('Original collision mesh changed; no output written.')
count = struct.unpack_from('<I', data, 80)[0]
if count != 3656 or len(data) != 84 + count * 50:
    raise SystemExit('Unexpected source STL layout.')
marking = json.loads((scene / 'flat-marking-contact-manifest.json').read_text())
central = set(marking['removed_triangle_indices'])
if len(central) != 60 or marking['source_sha256'] != expected:
    raise SystemExit('Central marking manifest mismatch.')
regions = [[3.5, -4.0, 6.0, -2.0], [-6.0, 2.0, -3.5, 4.0]]

def selected(v):
    x, y, z = v
    return abs(z - .003) < 1e-6 and any(a - 1e-6 <= x <= c + 1e-6 and bb - 1e-6 <= y <= dd + 1e-6 for a, bb, c, dd in regions)

def covers(x, y, tri):
    a, bb, c = tri
    den = (bb[1] - c[1]) * (a[0] - c[0]) + (c[0] - bb[0]) * (a[1] - c[1])
    if abs(den) < 1e-12:
        return False
    u = ((bb[1] - c[1]) * (x - c[0]) + (c[0] - bb[0]) * (y - c[1])) / den
    v = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / den
    return min(u, v, 1 - u - v) >= -1e-6

kept = []
changed = []
removed = []
vertices = set()
old_floor = []
new_floor = []
unchanged = 0
for index in range(count):
    block = data[84 + index * 50:84 + (index + 1) * 50]
    values = struct.unpack('<12fH', block)
    original = [list(values[k:k + 3]) for k in (3, 6, 9)]
    if index in central:
        if not all(abs(v[0]) <= 1.500001 and abs(v[1]) <= 1.500001 and .006999 <= v[2] <= .010001 for v in original):
            raise SystemExit('Central face selection changed.')
        continue
    tri = [v[:] for v in original]
    touched = False
    for v in tri:
        if selected(v):
            vertices.add(tuple(v))
            v[2] = 0.0
            touched = True
    if not touched:
        kept.append(block)
        unchanged += 1
        continue
    changed.append(index)
    if all(abs(v[2] - .003) < 1e-6 for v in original):
        old_floor.append(original)
        new_floor.append(tri)
    a, bb, c = tri
    u = [bb[k] - a[k] for k in range(3)]
    v = [c[k] - a[k] for k in range(3)]
    normal = [u[1]*v[2] - u[2]*v[1], u[2]*v[0] - u[0]*v[2], u[0]*v[1] - u[1]*v[0]]
    norm = math.sqrt(sum(x*x for x in normal))
    if norm < 1e-10:
        removed.append(index)
        continue
    normal = [x / norm for x in normal]
    kept.append(struct.pack('<12fH', *normal, *(x for v in tri for x in v), values[12]))
if (len(changed), len(vertices), len(removed), len(kept), unchanged) != (270, 82, 16, 3580, 3326):
    raise SystemExit('Floor selection count mismatch; no output written.')
checks = 0
covered = 0
for a, bb, c, dd in regions:
    for ix in range(round((c - a) / .05)):
        for iy in range(round((dd - bb) / .05)):
            x, y = a + (ix + .5) * .05, bb + (iy + .5) * .05
            checks += 1
            if any(covers(x, y, tri) for tri in old_floor):
                covered += 1
                if not any(covers(x, y, tri) for tri in new_floor):
                    raise SystemExit('Floor support lost; no output written.')
output = scene / 'models/rmul_2027_3v3_flat_floor'
mesh = b'RMUL2027 central marking and millimetric floor seam contact approximation'.ljust(80, b' ') + struct.pack('<I', len(kept)) + b''.join(kept)
mesh_path = output / 'meshes/collision/part_000.stl'
mesh_path.parent.mkdir(parents=True, exist_ok=True)
mesh_path.write_bytes(mesh)
model = ET.parse(scene / 'models/rmul_2027_3v3_flat_marking/model.sdf')
model.getroot().find('model').set('name', 'rmul_2027_3v3_flat_floor')
model.getroot().find("model/link/collision[@name='collision_part_000']/geometry/mesh/uri").text = 'model://rmul_2027_3v3_flat_floor/meshes/collision/part_000.stl'
ET.indent(model)
model.write(output / 'model.sdf', encoding='utf-8', xml_declaration=True)
(output / 'model.config').write_text('<?xml version="1.0"?><model><name>RMUL2027 flat floor contact approximation</name><version>1.0.0</version><sdf version="1.7">model.sdf</sdf><description>Central marking visual-only and two 3mm floor seam contacts flattened; original CAD and visuals retained. Simulation contact approximation, not official construction confirmation.</description></model>\n')
world = ET.parse(scene / 'worlds/rmul_2027_3v3_flat_marking_world.sdf')
world.getroot().find('world/include/uri').text = 'model://rmul_2027_3v3_flat_floor'
ET.indent(world)
world.write(scene / 'worlds/rmul_2027_3v3_flat_floor_world.sdf', encoding='utf-8', xml_declaration=True)
manifest = {'purpose': 'Simulation floor-contact approximation; not official construction confirmation', 'source_mesh': str(source.relative_to(folder)), 'source_sha256': expected, 'collision_mesh': str(mesh_path.relative_to(folder)), 'collision_sha256': hashlib.sha256(mesh).hexdigest(), 'central_marking_removed_faces': 60, 'floor_regions_world_xy_m': regions, 'floor_vertex_z_old_m': .003, 'floor_vertex_z_new_m': 0.0, 'floor_vertices_moved': len(vertices), 'floor_faces_affected': len(changed), 'degenerate_floor_side_faces_removed': len(removed), 'retained_faces': len(kept), 'unchanged_triangle_records': unchanged, 'changed_source_triangle_indices': changed, 'removed_degenerate_source_triangle_indices': removed, 'floor_grid_samples_checked': checks, 'previously_supported_samples_still_supported': covered, 'floor_support_lost_samples': 0, 'visual_meshes_unchanged': True, 'other_collision_meshes_unchanged': True, 'navigation_maps_unchanged': True, 'higher_vertices_unchanged': True, 'original_cad_retained': True}
(scene / 'flat-floor-contact-manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
print(json.dumps({key:value for key,value in manifest.items() if not key.endswith('triangle_indices')}, indent=2))
