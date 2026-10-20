import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.ndimage import binary_erosion, label
from scipy.spatial.transform import Rotation

sys.path.append('/usr/lib/freecad/lib')
import FreeCAD
import Mesh

root=Path(__file__).resolve().parents[4]
folder=Path(__file__).resolve().parent
artifact=root/'artifacts/rmul_2027_3v3'
output=artifact/'maps-cad'
output.mkdir(exist_ok=True)
preview=np.load(artifact/'height_preview.npz')
height=preview['height']
assert height.shape[0]%2==0 and height.shape[1]%2==0
maximum=height.reshape(height.shape[0]//2,2,height.shape[1]//2,2).max(axis=(1,3))
minimum=height.reshape(height.shape[0]//2,2,height.shape[1]//2,2).min(axis=(1,3))
free=(maximum<=.06)&(minimum>=-.02)
image=np.where(free,254,0).astype(np.uint8)
resolution=.05
origin=[float(preview['xs'][0]-.0125+4.8),float(preview['ys'][0]-.0125-3.0),0.0]
pgm=output/'rmul_2027_3v3.pgm'
pgm.write_bytes(f'P5\n{image.shape[1]} {image.shape[0]}\n255\n'.encode()+np.flipud(image).tobytes())
configuration={'image':pgm.name,'mode':'trinary','resolution':resolution,'origin':origin,'negate':0,'occupied_thresh':.65,'free_thresh':.196}
(output/'rmul_2027_3v3.yaml').write_text(yaml.safe_dump(configuration,sort_keys=False))
footprint_clear=binary_erosion(free,structure=np.ones((13,15),dtype=bool))
components,count=label(footprint_clear)
checkpoints={}
for name,(x,y) in {'spawn':(0.,0.),'west_far':(-.03,-5.86),'east_far':(9.6,-5.8),'east_north':(9.6,0.)}.items():
    col=int(math.floor((x-origin[0])/resolution));row=int(math.floor((y-origin[1])/resolution))
    checkpoints[name]={'map_xy':[x,y],'component':int(components[row,col]),'free':bool(free[row,col])}
assert checkpoints['spawn']['component']>0
assert all(p['component']==checkpoints['spawn']['component'] for p in checkpoints.values()),checkpoints
rng=np.random.default_rng(20271019)
points=[]
for path in sorted((folder/'scene/models/rmul_2027_3v3/meshes/collision').glob('*.stl')):
    vertices,faces=Mesh.Mesh(str(path)).Topology
    vertices=np.array([list(p) for p in vertices])
    for face in faces:
        tri=vertices[list(face)]
        if tri[:,2].mean()<-.005:
            continue
        area=np.linalg.norm(np.cross(tri[1]-tri[0],tri[2]-tri[0]))*.5
        size=max(3,int(math.ceil(area/(.035*.035))))
        uv=rng.random((size,2))
        flip=uv.sum(axis=1)>1
        uv[flip]=1-uv[flip]
        points.append(tri[0]+uv[:,0:1]*(tri[1]-tri[0])+uv[:,1:2]*(tri[2]-tri[0]))
world=np.vstack(points)
world=world[world[:,2]>=-.005]
indices=np.unique(np.floor(world/.025).astype(np.int32),axis=0,return_index=True)[1]
world=world[indices]
map_points=world+np.array([4.8,-3.,0.])
static=json.loads((artifact/'route-display-nav-20261019/tf.json').read_text())['base_footprint->front_mid360']
rotation=Rotation.from_quat(static['rotation_xyzw']).as_matrix()
translation=np.array(static['translation'])
raw=(map_points-translation)@rotation
assert np.max(np.abs(raw@rotation.T+translation-map_points))<1e-9
raw=raw.astype('<f4')
header=f'# CAD surface prior; inverse base_footprint-to-lidar calibration applied\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(raw)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(raw)}\nDATA binary\n'
(output/'rmul_2027_3v3.pcd').write_bytes(header.encode()+raw.tobytes())
manifest={'source':'Official CAD collision mesh surface sampling and conservative height rasterization; not a sensor SLAM map','source_sha256':'7d47c1b6a6a0253a732e46cb122bb84410b11d5134bcb5658104299db4a48ca4','map_resolution_m':resolution,'origin':origin,'grid_shape':list(free.shape),'free_cells':int(free.sum()),'occupied_cells':int((~free).sum()),'unknown_cells':0,'walkable_surface_height_m':[-.02,.06],'elevated_platforms_and_voids':'blocked; ramp/platform traversal not enabled','world_to_map_xyz':[4.8,-3.,0.],'pcd_points':len(raw),'pcd_encoding':'xyz float32 binary; existing loader applies the calibrated base-to-lidar transform once','calibration':static,'traversable_components':count,'checkpoints':checkpoints,'simulation_truth_as_localization_input':False}
manifest['file_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir() if p.suffix in ['.pcd','.pgm','.yaml']}
(output/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
print(json.dumps(manifest,ensure_ascii=False))
