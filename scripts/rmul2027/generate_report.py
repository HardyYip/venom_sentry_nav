import json
import math
import shutil
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager

root=Path('/home/hardy/pb2025_nav_ws')
artifact=root/'artifacts/rmul_2027_3v3'
folder=Path(__file__).resolve().parent
out=Path('/mnt/c/Users/Hardy/Documents/Codex/2026-10-18/headoff-md1/outputs')
index=json.loads((artifact/'final-far-index.json').read_text())
names=['far-seam-center','far-seam-northeast','far-seam-east-south',index['direct_record']]
labels=['东南→中央','中央→东北','东北→东南','重启后直接到东北']
def events(name):
    return [json.loads(line) for line in (artifact/name/'verify.jsonl').read_text().splitlines() if line.startswith('{')]
def event(name,kind):
    return next(e for e in events(name) if e['event']==kind)
for name in names+['far-seam-invalid','ready-far-final','baseline-regression-final']:
    assert any(e['event']=='PASS' for e in events(name)),name
manifest=json.loads((artifact/'maps-cad/manifest.json').read_text())
maps=json.loads((artifact/names[-1]/'maps.json').read_text())['map']
grid=np.asarray(maps['data']).reshape(maps['height'],maps['width'])
resolution=maps['resolution'];origin=np.array(maps['origin'])
occupied=np.column_stack(np.where(grid==100))[:,::-1]*resolution+origin+resolution/2
tree=cKDTree(occupied)
traces={};rows=[]
for name,label in zip(names,labels):
    trace=json.loads((artifact/name/'trace.json').read_text())
    gt=[e for e in trace if e['kind']=='ground_truth']
    xy=np.array([e['value'][:2] for e in gt])+[4.8,-3.0]
    traces[name]=xy
    distance=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum())
    last=None;errors=[];ages=[]
    stamps=[e['stamp'] for e in gt]
    for e in trace:
        if e['kind']=='ground_truth':last=e
        elif e['kind']=='estimated_pose' and last is not None:
            if 'stamp' in e and stamps[0]<=e['stamp']<=stamps[-1]:
                ref=[np.interp(e['stamp'],stamps,xy[:,k]) for k in range(2)]
                ages.append(last['stamp']-e['stamp'])
            else:ref=np.array(last['value'][:2])+[4.8,-3.0]
            errors.append(float(np.linalg.norm(np.array(e['value'][:2])-ref)))
    rtf=(gt[-1]['stamp']-gt[0]['stamp'])/(gt[-1]['wall']-gt[0]['wall'])
    row={'record':name,'label':label,'goal':event(name,'goal')['xy'],'path_length_m':distance,'max_localization_error_m':max(errors),'max_tf_age_s':max(ages) if ages else None,'sample_real_time_factor':rtf,'result':event(name,'result'),'physical_pose':event(name,'physical_pose')}
    if name==names[-1]:
        bounds=[]
        for e,p in zip(gt,xy):
            candidates=tree.query_ball_point(p,1.)
            if not candidates:continue
            delta=occupied[candidates]-p;c,s=math.cos(e['yaw']),math.sin(e['yaw'])
            q=np.abs(delta@np.array([[c,-s],[s,c]]))-[.31,.27]
            sd=np.linalg.norm(np.maximum(q,0),axis=1)+np.minimum(np.max(q,axis=1),0)
            bounds.append(float(sd.min()-resolution/math.sqrt(2)))
        row['padded_footprint_clearance_lower_bound_m']=min(bounds)
    rows.append(row)
invalid=event('far-seam-invalid','invalid_result')
stable=event('ready-far-final','localization_stability')
baseline=event('baseline-regression-final','result')
tf=json.loads((artifact/'ready-far-final/tf.json').read_text())
assert not any('error' in value for value in tf.values())
run=Path(index['run_log']);options=json.loads((run/'run-options.json').read_text())
nav_log=(run/'navigation.log').read_text()
assert 'Running backup' not in nav_log, 'Final suite unexpectedly ran backup recovery'
assert options['map_set']=='cad' and options['wheel_contact']=='rounded'
summary={'date':'2026-10-19','scope':'Static CAD field representative long-distance navigation; not real robot validation','map_manifest':manifest,'default_options':options,'navigation_tests':rows,'occupied_goal':invalid,'stationary_localization':stable,'baseline_regression':baseline,'tf':tf,'run_logs':[str(run)],'simulation_truth_as_localization_input':False,'physics_profile':'32 spherical roller envelopes per original wheel and 6cm collision-only transitions at 12 low CAD floor seams; navigation approximation', 'navigation_contact_scene':json.loads((folder/'scene/navigation-contact-manifest.json').read_text()),'limitations':['Original vertical CAD step traversal not validated; navigation world adds explicit seam transitions','No real mecanum dynamics calibration','Elevated platforms and ramps blocked','No dynamic competition obstacle validation','No same-field comparison with old Venom TEB']}
(artifact/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
font=Path('/mnt/c/Windows/Fonts/msyh.ttc');font_manager.fontManager.addfont(str(font))
plt.rcParams['font.family']=font_manager.FontProperties(fname=str(font)).get_name();plt.rcParams['axes.unicode_minus']=False
fig,ax=plt.subplots(figsize=(12,8),layout='constrained')
ax.imshow(grid,origin='lower',extent=[origin[0],origin[0]+grid.shape[1]*resolution,origin[1],origin[1]+grid.shape[0]*resolution],cmap='Greys',vmin=0,vmax=100,interpolation='nearest',alpha=.72)
for row,color in zip(rows,['#e5811f','#2272bf','#198755','#d12e9a']):
    xy=traces[row['record']]
    ax.plot(xy[:,0],xy[:,1],color=color,lw=2.5,label=f"{row['label']}：{row['path_length_m']:.1f} m，误差 {row['result']['ground_truth_goal_error']:.2f} m")
    ax.scatter(*row['goal'],marker='*',s=160,color=color,edgecolor='white',linewidth=.7,zorder=5)
ax.scatter(0,0,s=100,facecolor='white',edgecolor='black',zorder=6);ax.annotate('出生点',(0,0),xytext=(.2,.35),fontsize=11)
ax.set_title('完整 CAD 导航地图与实际行驶轨迹\n低接缝采用斜面碰撞近似；四次到点均成功并停车，星号为目标',fontsize=15,pad=13)
ax.set_xlabel('map X / m');ax.set_ylabel('map Y / m');ax.set_aspect('equal');ax.legend(loc='lower left',fontsize=10,framealpha=.96);ax.grid(alpha=.16)
fig.savefig(artifact/'far-navigation-result.png',dpi=170);plt.close(fig)
fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
for ax,step in zip(axes,[.011,.003]):
    ax.plot([-10,0,0,5],[0,0,step*1000,step*1000],color='#34383b',lw=2.5,label='原 CAD 竖直接缝')
    ax.plot([-7.5,-1.5,2.5],[-.1,step*1000+.1,step*1000+.1],color='#d12e9a',lw=2.5,label='导航碰撞过渡')
    ax.set_title(f'{step*1000:.0f} 毫米地面高差');ax.set_xlabel('距原接缝 / cm');ax.set_ylabel('高度 / mm');ax.grid(alpha=.2);ax.legend(fontsize=9)
fig.suptitle('导航场景的碰撞近似：保留高差，增加窄斜面；原 CAD 场景独立保留',fontsize=12)
fig.savefig(artifact/'contact-approximation.png',dpi=170);plt.close(fig)
lines=['| 路线 | 实际轨迹长度 | 墙钟耗时 | 真值终点误差 | 停车速度最大分量 |','| --- | ---: | ---: | ---: | ---: |']
for row in rows:
    v=row['result'];lines.append(f"| {row['label']} | {row['path_length_m']:.2f} m | {v['elapsed_wall_s']:.2f} s | {v['ground_truth_goal_error']:.3f} m | {v['stopped_speed']:.5f} |")
report=f'''# 远距离导航修复与验收记录

2026-10-19，Asia/Shanghai。当前默认为完整 CAD 导航地图、rounded 轮胎接触及 12 段低接缝斜面过渡场景。原 CAD 碰撞场景保留，不把近似场景的成功当作原竖直台阶通过能力。四条代表性路线返回 SUCCEEDED（状态 4），终点估计及真值误差均≤0.35 米，最终 cmd_vel=0、实际底盘停稳。四条路线均未触发退让恢复。

{chr(10).join(lines)}

停车速度列取 |vx|、|vy|、|wz| 最大分量，线速度单位 m/s、角速度单位 rad/s，均小于 0.03。轨迹长度由真值采样点累计计算，区别于起终点直线位移。最后一项为默认入口重启后直接发远点，无人工中间点。实际轨迹图为 far-navigation-result.png，碰撞近似剖面图为 contact-approximation.png；原始轨迹、路径、代价地图、TF、物理姿态及日志均保存在 D 盘 WSL。

## 原因与修改

1. 原传感器地图只有约 21% 已知，PCD 也只覆盖附近，远目标发生规划失败或重定位不收敛。新增独立 maps-cad，由官方碰撞网格保守栅格化并采样完整 PCD；原 maps 保留。当前分辨率 0.05 米，254×174 格，空闲 {manifest['free_cells']:,}、占用 {manifest['occupied_cells']:,}，PCD {manifest['pcd_points']:,} 点。完整已知来自 CAD，不是传感器全场 SLAM，也没有把原未知格直接改白。
2. 原完整圆柱轮碰撞侧缘会卡在中央约 10 毫米地面边缘。新增可选每轮 32 个球形滚轮包络，保留原质量、惯性、轮心、轮径、宽度和外观。场景中央约 10–11 毫米、右侧约 3 毫米高差均保留。这仍不能可靠通过原中央竖直台阶，far-aligned-direct 已因物理无进展取消。因此当前独立导航 world 从 CAD 高度缓存自动提取 12 段 3–11 毫米低接缝，额外加水平长度 6 厘米的碰撞斜面，低侧提前 1.5 厘米、高侧重叠 2.5 厘米、顶面余量 0.1 毫米，不降低场地高差，不填孔洞，不改变墙和高台；原 CAD world/mesh 保留，--field-contact cad 可复现。这些是导航碰撞近似，不是官方原貌或真实麦克纳姆轮动力学验收。
3. 已读取 Gazebo 实际 cmd_vel，确认小高差前的卡滞不是指令丢失。原底盘以速度误差产生机身作用力，低速和地面边缘出现静摩擦/稳态误差。2027 启用可选 PI 和力限幅，并在轴向零速指令清积分；旧场景默认比例增益、圆柱轮和力限幅保留。未提高导航 0.5 m/s 限速。
4. 2027 独立行为树将规划失败与执行失败分开：无效规划清全局图后重试一次，不退让；执行失败才可清局部图并退让 0.2 米，最多两次。占用目标完整 NavigateToPose 返回 ABORTED（状态 {invalid['status']}），非零速度指令 {invalid['nonzero_commands']} 条，位移 {invalid['ground_truth_moved']:.6f} 米，{invalid['elapsed_wall_s']:.2f} 秒结束。
5. 工业相机降为 640×360@5，雷达、IMU 和定位下采样配置保留。直接远点采样实时倍率约 {rows[-1]['sample_real_time_factor']:.2f}；接近目标仍会减速。
6. 运动中定位评估按 TF 时间戳插值对齐真值，避免将不同时刻的位置相减。精度阈值仍为 0.35 米；另检查定位 TF 持续过旧和有指令却 8 秒不动。真值仅用于验收，不输入定位或控制。

## 验证与边界

重启直接远点的最大采样定位误差 {rows[-1]['max_localization_error_m']:.3f} 米；带 padding 的矩形足迹相对占用栅格最小保守间距下界 {rows[-1]['padded_footprint_clearance_lower_bound_m']:.3f} 米。该检查基于离散位置、实际航向和栅格，不能替代完整物理接触记录。

最终静止检查：最大平面漂移 {stable['max_xy_drift_m']:.3f} 米，真值误差 {stable['ground_truth_error_m']:.3f} 米。map→odom→base_footprint 及雷达、云台 TF 连通，核心 Nav2 节点 active 且单实例，时钟、传感器和里程计持续更新。

原 rmuc_2025 启动入口回归 SUCCEEDED，位移 {baseline['ground_truth_moved']:.3f} 米，估计终点误差 {baseline['goal_error']:.3f} 米。底盘插件 Release 编译、Python 语法与三个仓库 git diff --check 通过。旧 Venom 工作区未改动，未提交或推送。

当前仅验证带低台阶斜面近似的静态场地代表性远点与不可达目标拒绝；原竖直 CAD 台阶通过能力仍未解决。高台、坡道、孔洞按障碍处理；未验收所有点、动态道具、比赛战术或 Venom 实车。small_gicp 偶发不收敛告警仍可能出现。旧 Venom 仿真使用 TEB，尚无同场景同机器人同限速对比，不能由此给导航算法排名。

## 证据位置

工程：/home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027。

产物：/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3。当前记录为 {', '.join(names)}、far-seam-invalid、ready-far-final、baseline-regression-final。最终运行日志：{run}。每次运行保留参数快照及实际机器人 SDF，地图来源与哈希见 maps-cad/manifest.json。

旧局部地图、初轮报告及失败过程全部保留；historical-local-acceptance 为历史报告。被拒绝的 ellipsoid 模型、原 CAD 场景卡滞和异步定位评估取消均保留，不作为当前通过依据。

[官方场地来源](https://bbs.robomaster.com/article/1970956?source=4)。操作与逐终端命令见 操作说明.md。上游许可证、作者、历史和旧兼容补丁保留。
'''
(folder/'TEST_RESULTS.zh-CN.md').write_text(report)
out.mkdir(parents=True,exist_ok=True)
for source,target in [(folder/'README.zh-CN.md',out/'操作说明.md'),(folder/'TEST_RESULTS.zh-CN.md',out/'验收记录.md'),(artifact/'summary.json',out/'summary.json'),(artifact/'far-navigation-result.png',out/'far-navigation-result.png'),(artifact/'contact-approximation.png',out/'contact-approximation.png')]:shutil.copyfile(source,target)
(out/'地图与卡墙排查.md').write_text('''# 地图与卡墙问题：2026-10-19 更新

旧传感器局部地图约 21% 覆盖，已保留。当前默认使用完整 CAD 栅格与 PCD；早先 full-field-vs-map.png 仅用于历史对照。

本次补全地图，调整接触和速度环，并为12 段低接缝添加独立的碰撞斜面近似；原 CAD 竖直台阶的横移卡滞仍保留为限制。无效目标不再反复回退。结果、限制与失败记录说明见 验收记录.md，实际轨迹见 far-navigation-result.png。

粉色线来自规划器。墙内目标应被拒绝，不会有有效到点路径；已验证代表性可通行远点。
''')
(out/'回退速度与路径显示.md').write_text(f'''# 回退、速度与规划路径：2026-10-19 更新

无效规划只清全局地图重试一次；执行失败才可退让 0.2 米，最多两次。墙内目标实测无非零指令、无位移，返回 ABORTED。

速度上限仍为 0.5 m/s，轻量相机后直接远点实时倍率约 {rows[-1]['sample_real_time_factor']:.2f}；接近目标会减速。

RViz 顶层“到目标的规划路径”为粉色 /red_standard_robot1/plan，蓝色为局部路径。当前完整导航地图来自 CAD；Official CAD Reference 是额外三维参考，可单独关闭。

当前结果以 验收记录.md 为准，旧局部地图与旧参数说明已被本次更新取代。
''')
print(json.dumps({'direct_path_m':rows[-1]['path_length_m'],'direct_error_m':rows[-1]['result']['ground_truth_goal_error'],'direct_clearance_m':rows[-1]['padded_footprint_clearance_lower_bound_m'],'outputs':str(out)},ensure_ascii=False))
