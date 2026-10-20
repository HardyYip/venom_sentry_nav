import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.colors import LinearSegmentedColormap, Normalize
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
sys.path.append('/usr/lib/freecad/lib')
import FreeCAD
import Mesh

font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/msyh.ttc')
plt.rcParams.update({'font.family':font_manager.FontProperties(fname='/mnt/c/Windows/Fonts/msyh.ttc').get_name(),'axes.unicode_minus':False,'font.size':12})
base=Path(__file__).resolve().parent
art=Path('/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3')
triangles=[]
for path in sorted((base/'scene/models/rmul_2027_3v3/meshes/visual').glob('*.stl')):
    vertices,faces=Mesh.Mesh(str(path)).Topology
    vertices=np.array([list(p) for p in vertices])
    for face in faces:
        triangle=vertices[list(face)]
        if triangle[:,2].max()>=-.001:
            triangles.append(triangle)
triangles=np.array(triangles)
cmap=LinearSegmentedColormap.from_list('field_height',[(0,'#becfd8'),(.2/.42,'#f1cb65'),(.30/.42,'#b88a51'),(1,'#d0b4a1')])
colors=cmap(Normalize(0,.42)(triangles[:,:,2].mean(axis=1)))
normals=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0])
normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-12)
light=np.array([-.4,-.5,.77]);light/=np.linalg.norm(light)
shade=.65+.35*np.abs(normals@light)
colors[:,:3]*=shade[:,None]
fig=plt.figure(figsize=(15,7.6),facecolor='white')
gs=fig.add_gridspec(1,2,width_ratios=[1.3,1])
ax=fig.add_subplot(gs[0,0],projection='3d')
collection=Poly3DCollection(triangles,facecolors=colors,edgecolors=(.25,.3,.33,.10),linewidths=.15,zsort='average')
ax.add_collection3d(collection)
ax.set(xlim=(-6.4,6.4),ylim=(-4.4,4.4),zlim=(-.05,.47),xlabel='世界 X（米）',ylabel='世界 Y（米）',zlabel='高度（米）')
ax.set_box_aspect((12.8,8.8,1.56))
ax.view_init(elev=44,azim=-63)
ax.scatter([-4.8],[3.0],[.1],color='#e83f58',s=75,depthshade=False)
ax.text(-4.8,3.0,.22,'出生点',color='#a80c2d',fontsize=12)
ax.set_title('官方 STEP 转换出的三维场地\n为看清高差，竖直方向放大 3 倍',pad=12,fontsize=14)
ax.grid(False)
for axis in (ax.xaxis,ax.yaxis,ax.zaxis):axis.pane.fill=False
ax.set_zticks([0,.2,.4])
right=fig.add_subplot(gs[0,1])
data=np.load(art/'height_preview.npz');height=data['height'];xs=data['xs'];ys=data['ys'];r=float(data['resolution'])
right.imshow(height,origin='lower',extent=[xs[0]-r/2,xs[-1]+r/2,ys[0]-r/2,ys[-1]+r/2],cmap=cmap,vmin=0,vmax=.42)
right.scatter([-4.8],[3],s=80,color='#e83f58',edgecolors='white',zorder=6)
right.annotate('机器人出生点',xy=(-4.8,3),xytext=(-4.6,1.7),color='#a80c2d',arrowprops={'arrowstyle':'->','color':'#a80c2d'},fontsize=12)
for point,text in [((-2.5,.6),'约 0.20 米高台'),((2.5,-1.9),'约 0.20 米高台')]:
    right.text(*point,text,ha='center',va='center',rotation=90,fontsize=11,color='#453112')
right.set(xlabel='世界 X（米）',ylabel='世界 Y（米）',title='同一场地俯视图（按高度着色）')
right.set_aspect('equal')
fig.suptitle('RMUL 2027 3V3 V1.0.0 官方场地模型',fontsize=21,y=.96)
fig.text(.5,.065,'这是官方场地几何。RViz 的黑白导航地图由机器人传感器建图生成，当前只覆盖部分区域。',ha='center',fontsize=13,color='#333333')
fig.subplots_adjust(left=.025,right=.97,bottom=.14,top=.84,wspace=.08)
path=art/'official-field-explained.png'
fig.savefig(path,dpi=140,facecolor='white')
plt.close(fig)
print(path)
