import json
import sys
from pathlib import Path

sys.path.append('/usr/lib/freecad/lib')
import FreeCAD
import Mesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

folder = Path(__file__).resolve().parent
scene = folder / 'scene'
resolution = .025
xmin, xmax, ymin, ymax = -6.35, 6.35, -4.35, 4.35
xs = np.arange(xmin + resolution/2, xmax, resolution)
ys = np.arange(ymin + resolution/2, ymax, resolution)
height = np.full((len(ys), len(xs)), -.2, dtype=np.float32)
for path in (scene / 'models/rmul_2027_3v3/meshes/visual').glob('*.stl'):
    mesh = Mesh.Mesh(str(path))
    vectors, facets = mesh.Topology
    vertices = np.array([list(p) for p in vectors])
    for facet in facets:
        tri = vertices[list(facet)]
        mat = np.array([[tri[0,0]-tri[2,0], tri[1,0]-tri[2,0]],
                        [tri[0,1]-tri[2,1], tri[1,1]-tri[2,1]]])
        if abs(np.linalg.det(mat)) < 1e-10:
            continue
        cols = np.flatnonzero((xs >= tri[:,0].min()-resolution) & (xs <= tri[:,0].max()+resolution))
        rows = np.flatnonzero((ys >= tri[:,1].min()-resolution) & (ys <= tri[:,1].max()+resolution))
        if not len(cols) or not len(rows):
            continue
        xx, yy = np.meshgrid(xs[cols], ys[rows])
        uv = np.linalg.solve(mat, np.stack([xx.ravel()-tri[2,0], yy.ravel()-tri[2,1]]))
        a, b = uv.reshape(2, len(rows), len(cols))
        mask = (a >= -1e-5) & (b >= -1e-5) & (a+b <= 1+1e-5)
        z = tri[2,2] + a*(tri[0,2]-tri[2,2]) + b*(tri[1,2]-tri[2,2])
        old = height[np.ix_(rows, cols)]
        height[np.ix_(rows, cols)] = np.where(mask, np.maximum(old,z), old)
output = Path('/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3')
np.savez_compressed(output/'height_preview.npz', height=height, xs=xs, ys=ys, resolution=resolution)
fig, ax = plt.subplots(figsize=(12,8))
plot = ax.imshow(height, origin='lower', extent=[xmin,xmax,ymin,ymax], vmin=0, vmax=.42, cmap='terrain')
ax.scatter([-4.8],[3.0],s=100,c='#ff3050',edgecolors='white',label='Robot spawn')
ax.arrow(-4.8,3,.8,0,width=.025,color='#ff3050',length_includes_head=True)
ax.set(xlabel='World X (m)',ylabel='World Y (m)',title='Official RMUL 2027 3V3 V1.0.0 — CAD height preview')
ax.legend(loc='upper right')
fig.colorbar(plot,ax=ax,label='Top surface Z (m)')
fig.tight_layout()
fig.savefig(output/'field-preview.png',dpi=150)
plt.close(fig)
print(json.dumps({'preview':str(output/'field-preview.png'),'spawn_surface_height_m':float(height[np.argmin(abs(ys-3)),np.argmin(abs(xs+4.8))])}))
