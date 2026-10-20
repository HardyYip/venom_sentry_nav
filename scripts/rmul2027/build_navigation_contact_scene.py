from pathlib import Path
from collections import Counter,defaultdict
import json
import numpy as np
import xml.etree.ElementTree as ET

folder=Path(__file__).resolve().parent
artifact=folder.parents[3]/'artifacts/rmul_2027_3v3'
preview=np.load(artifact/'height_preview.npz')
h=preview['height'];xs=preview['xs'];ys=preview['ys'];spacing=float(preview['resolution'])
width=.06;low_margin=.015;high_margin=.025
segments=defaultdict(list)
for axis in ['x','y']:
    first=h[:,:-1] if axis=='x' else h[:-1,:]
    second=h[:,1:] if axis=='x' else h[1:,:]
    low=np.minimum(first,second);high=np.maximum(first,second)
    valid=np.isfinite(first)&np.isfinite(second)&(low>=-.001)&(high<=.02)&(high-low>=.0015)
    for row,col in zip(*np.where(valid)):
        boundary=float((xs[col]+xs[col+1])/2 if axis=='x' else (ys[row]+ys[row+1])/2)
        direction=1 if second[row,col]>first[row,col] else -1
        tangent=float(ys[row] if axis=='x' else xs[col])
        key=(axis,round(boundary,6),direction,round(float(low[row,col]),6),round(float(high[row,col]),6))
        segments[key].append((tangent-spacing/2,tangent+spacing/2))
vertices=[];faces=[];records=[]
for key,parts in sorted(segments.items()):
    merged=[]
    for start,end in sorted(parts):
        if merged and abs(merged[-1][1]-start)<1e-5:merged[-1][1]=end
        else:merged.append([start,end])
    axis,boundary,direction,low,high=key
    cross=[(-width-low_margin,low-.0001),(-low_margin,high+.0001),(high_margin,high+.0001),(high_margin,low-.005),(-width-low_margin,low-.005)]
    for start,end in merged:
        prism=[]
        for tangent in [start,end]:
            for distance,z in cross:
                normal=boundary+direction*distance
                prism.append([normal,tangent,z] if axis=='x' else [tangent,normal,z])
        prism=np.array(prism)
        region=(xs>=prism[:,0].min())&(xs<=prism[:,0].max())
        rows=(ys>=prism[:,1].min())&(ys<=prism[:,1].max())
        support=h[np.ix_(rows,region)]
        assert support.size and np.all(np.isfinite(support)) and np.min(support)>=-.02, key
        pf=[]
        for i in range(1,4):pf.extend([[0,i+1,i],[5,5+i,5+i+1]])
        for i in range(5):
            j=(i+1)%5;pf.extend([[i,j,j+5],[i,j+5,i+5]])
        pf=np.asarray(pf)
        volume=sum(np.dot(prism[f[0]],np.cross(prism[f[1]],prism[f[2]])) for f in pf)/6
        if volume<0:pf=pf[:,[0,2,1]]
        edges=Counter(tuple(sorted((int(f[i]),int(f[(i+1)%3])))) for f in pf for i in range(3))
        assert all(count==2 for count in edges.values())
        offset=len(vertices);vertices.extend(prism.tolist());faces.extend((pf+offset).tolist())
        records.append({'axis':axis,'boundary_m':boundary,'high_direction':direction,'tangent_range_m':[start,end],'low_m':low,'high_m':high})
vertices=np.asarray(vertices)
lines=['solid navigation_seam_transition']
for f in faces:
    triangle=vertices[f];normal=np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0]);normal/=np.linalg.norm(normal)
    lines.extend(['facet normal '+' '.join(map(str,normal)),'outer loop'])
    lines.extend('vertex '+' '.join(map(str,v)) for v in triangle);lines.extend(['endloop','endfacet'])
lines.append('endsolid navigation_seam_transition')
path=folder/'scene/models/rmul_2027_3v3/meshes/navigation_seam_transition.stl'
path.write_text('\n'.join(lines)+'\n')
world=ET.parse(folder/'scene/worlds/rmul_2027_3v3_world.sdf')
model=ET.SubElement(world.getroot().find('world'),'model',name='navigation_seam_transition');ET.SubElement(model,'static').text='true'
link=ET.SubElement(model,'link',name='seam_transition');collision=ET.SubElement(link,'collision',name='low_step_transitions')
ET.SubElement(ET.SubElement(ET.SubElement(collision,'geometry'),'mesh'),'uri').text='model://rmul_2027_3v3/meshes/navigation_seam_transition.stl'
ET.indent(world,space='  ');world.write(folder/'scene/worlds/rmul_2027_3v3_navigation_world.sdf',encoding='utf-8',xml_declaration=True)
manifest={'purpose':'Optional navigation collision approximation, not exact official CAD contact geometry','original_world':'rmul_2027_3v3_world.sdf','navigation_world':'rmul_2027_3v3_navigation_world.sdf','official_mesh_modified':False,'height_cache_resolution_m':spacing,'step_height_range_m':[.0015,.02],'slope_run_m':width,'low_side_margin_m':low_margin,'high_side_overlap_m':high_margin,'top_margin_m':.0001,'mesh_triangles':len(faces),'closed_prisms':len(records),'all_prisms_watertight':True,'support_validated_no_missing_floor':True,'segments':records,'limitations':'Retains floor height differences but bevels low vertical seams; no real mecanum step traversal claim'}
(folder/'scene/navigation-contact-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({k:v for k,v in manifest.items() if k!='segments'}))
