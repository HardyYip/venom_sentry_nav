"""Orthographic bounding-box diagram. Not a mesh/occlusion or collision proof."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np


def render(assembly, manifest, output):
    plane = manifest['wheel_reference_plane_cad_z_mm']
    rotation = np.diag([-1.,1.,-1.])
    offset = np.array([0.,0.,plane])

    def bounds(record):
        box = record['bbox_mm']
        points = np.array([[box[i] for i in range(3)], [box[i+3] for i in range(3)]])
        points = points @ rotation.T + offset
        return points.min(axis=0), points.max(axis=0)

    fig, axes = plt.subplots(1, 3, figsize=(15, 7), facecolor='#f7f9fc')
    fig.suptitle('C071 STEP geometry review | two MID360 units | NOT a runtime TF', fontsize=17, y=.97)
    views = [(0,1,'Plan view: X / Y'),(0,2,'Side projection: X / Z'),(1,2,'Front projection: Y / Z')]
    for ax, (i,j,title) in zip(axes, views):
        ax.set_facecolor('white')
        for row in assembly['records']:
            # Root's direct components only; bounding rectangles can overestimate solids.
            if len(row['path']) != 2 or not row.get('visible') or not row.get('bbox_mm'):
                continue
            low,high = bounds(row)
            if max(high-low) < 20:
                continue
            ax.add_patch(Rectangle((low[i],low[j]),high[i]-low[i],high[j]-low[j],
                                   fill=False,edgecolor='#9ba6b3',linewidth=.45,alpha=.4))
        for radar,color in zip(manifest['radars'],['#005db3','#d35400']):
            low,high = bounds({'bbox_mm':radar['combined_bbox_cad_mm']})
            ax.add_patch(Rectangle((low[i],low[j]),high[i]-low[i],high[j]-low[j],
                                   fill=False,edgecolor=color,linewidth=1.8))
            center = np.array(radar['reference_view_xyz_m'])*1000
            direction = rotation @ np.array(radar['geometric_aperture_axis_cad'])
            ax.plot(center[i],center[j],'o',color=color,markersize=5)
            ax.annotate('',xy=(center[i]+direction[i]*90,center[j]+direction[j]*90),
                        xytext=(center[i],center[j]),
                        arrowprops=dict(arrowstyle='->',color=color,lw=2))
            ax.annotate(radar['id']+' (60 deg)',(center[i],center[j]),
                        xytext=(7,10 if radar['id']=='primary' else -18),
                        textcoords='offset points',color=color,fontsize=9,
                        bbox=dict(facecolor='white',alpha=.8,edgecolor='none',pad=1))
        if j == 2:
            ax.axhline(0,color='#56697a',ls='--',lw=1)
            ax.text(-300,6,'static wheel reference',fontsize=8,color='#56697a')
            ax.set_ylim(-80,660)
        else:
            ax.set_ylim(-330,330)
        ax.set_xlim(-330,330)
        ax.set_aspect('equal',adjustable='box')
        ax.set_title(title,fontsize=12)
        ax.set_xlabel('XYZ'[i]+' [mm]')
        ax.set_ylabel('XYZ'[j]+' [mm]')
        ax.grid(alpha=.15)
    fig.text(.05,.12,'View coordinates: X = -CAD X; Y = CAD Y; Z = wheel-plane CAD Z - CAD Z.',
             fontsize=11)
    fig.text(.05,.08,'Markers = cover sphere centers, not calibrated scan origins. Arrows = geometric aperture axes.',
             fontsize=10)
    fig.text(.05,.04,'Gray = component bounding boxes, not exact silhouettes. Big-Yaw following relation remains UNCONFIRMED.',
             fontsize=10,color='#9a3a00')
    fig.subplots_adjust(left=.05,right=.98,bottom=.22,top=.88,wspace=.28)
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('assembly',type=Path)
    parser.add_argument('manifest',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    render(json.loads(args.assembly.read_text()),json.loads(args.manifest.read_text()),args.output)
