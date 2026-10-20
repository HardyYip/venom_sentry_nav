import json
import re
from pathlib import Path
import numpy as np

root=Path('/home/hardy/pb2025_nav_ws')
artifact=root/'artifacts/rmul_2027_3v3'
folder=root/'src/pb2025_sentry_nav/scripts/rmul2027'
def events(name):
    return [json.loads(line) for line in (artifact/name/'verify.jsonl').read_text().splitlines() if line.startswith('{')]
def event(name,kind):
    return next(e for e in events(name) if e['event']==kind)
for name in ['nav-relocalization','blocked-goal','ready-final','baseline-regression']:
    assert any(e['event']=='PASS' for e in events(name)),name
nav=event('nav-relocalization','result')
blocked=event('blocked-goal','blocked_result')
stable=event('ready-final','localization_stability')
baseline=event('baseline-regression','result')
physics=event('nav-relocalization','physical_pose')
cloud=(artifact/'maps/rmul_2027_3v3.pcd').read_bytes()
points=int(re.search(rb'POINTS (\d+)',cloud[:500]).group(1))
maps=json.loads((artifact/'nav-relocalization/maps.json').read_text())
unique,counts=np.unique(maps['map']['data'],return_counts=True)
cells={int(k):int(v) for k,v in zip(unique,counts)}
assert cells.get(-1,0)>0
transforms=json.loads((artifact/'ready-final/tf.json').read_text())
assert not any('error' in value for value in transforms.values())
ui_recovery=json.loads((artifact/'ui-recovery.json').read_text()) if (artifact/'ui-recovery.json').exists() else {}
ui_confirmed=ui_recovery.get('resolved',False) and bool(ui_recovery.get('visibility_evidence'))
route_display=json.loads((artifact/'route-display-nav-20261019/summary.json').read_text()) if (artifact/'route-display-nav-20261019/summary.json').exists() else {}
manual_wall=json.loads((artifact/'manual-wall/runtime.json').read_text()) if (artifact/'manual-wall/runtime.json').exists() else {}
ui_followup='RViz 当前可见性已由用户确认。' if ui_confirmed else 'RViz 当前可见性仍待用户确认；黑屏在窗口重开时曾复发。'
ui_status='User confirmed RViz map and robot visible after WSLg restart; headless Gazebo with one RViz' if ui_confirmed else 'RViz visibility confirmation pending'
ui_row='用户确认地图与机器人可见；WSLg 重启恢复，Gazebo 后台服务器 + 单个轻量 RViz' if ui_confirmed else '窗口可见性待用户确认'
summary={'navigation':nav,'blocked_goal':blocked,'stationary_localization':stable,
         'baseline_regression':baseline,'physical_pose':physics,
         'map_cell_counts':cells,'prior_pcd_points':points,'tf':transforms,
         'ui_status':ui_status,'ui_recovery':ui_recovery,'manual_wall_followup':manual_wall,'route_display_followup':route_display}
(artifact/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False)+'\n')
text=f'''# RMUL 2027 3V3 首轮仿真验收

日期：2026-10-18（Asia/Shanghai）。结论：官方新场景、北极熊机器人、静态地图重定位与短距离自主导航通过本轮自动验收。范围为出生点附近基础导航。

## 最终结果

| 项目 | 结果与证据 |
| --- | --- |
| 官方场地 | RMUL2027_3V3_V1.0.0.stp，1,989,786 字节，FreeCAD 实际打开成功 |
| 场景 | Fortress 独立加载、SDF Valid、时钟推进；可视/碰撞分开，各 3811 三角形 |
| 先验地图 | Point-LIO PCD {points:,} 点；PGM/YAML 来自仿真传感器 |
| 未知区域 | 加载后 {cells.get(-1,0):,} 个未知格、{cells.get(0,0):,} 个空闲格、{cells.get(100,0):,} 个占用格；未将未知格转换为空闲 |
| 目标点导航 | NavigateToPose 返回 SUCCEEDED；约 {nav['elapsed_wall_s']:.2f} 秒 |
| 真值运动 | {nav['ground_truth_moved']:.3f} 米 |
| 终点误差 | 估计 {nav['goal_error']:.3f} 米；独立真值 {nav['ground_truth_goal_error']:.3f} 米 |
| 停车 | 最终 cmd_vel 为 0；实际最大速度分量 {nav['stopped_speed']:.4f} |
| 物理姿态 | Gazebo 模型高度 {physics['xyz'][2]:.4f} 米，roll/pitch 接近零 |
| 静止定位 | 漂移 {stable['max_xy_drift_m']:.3f} 米，真值误差 {stable['ground_truth_error_m']:.3f} 米 |
| 阻塞目标 | 占用格目标被 ComputePathToPose 拒绝，路径点数 {blocked['path_points']}；机器人位移 {blocked['ground_truth_moved']:.6f} 米 |
| 原场地回归 | rmuc_2025 导航 SUCCEEDED，位移 {baseline['ground_truth_moved']:.3f} 米、估计终点误差 {baseline['goal_error']:.3f} 米 |
| 图形界面 | {ui_row} |

最终测试记录保存在 /home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3 下：

- nav-relocalization：最终重启定位导航、真值轨迹、路径、地图、TF、实际物理姿态。
- blocked-goal：最终占用目标拒绝及静止记录。
- ready-final：最终静止定位和单实例生命周期检查。
- baseline-regression：原 rmuc_2025 启动入口回归。
- mapping-final-goal、mapping-settled-ready：落地后初始化与建图运动验证。
- source/manifest.json、scene_test：官方来源校验与独立场景检查。

## 实现与坐标

工程与地图均保存在 D 盘 WSL 工作区 /home/hardy/pb2025_nav_ws。场景、网格和脚本位于源码仓库 scripts/rmul2027；大地图、原始 STEP 和日志位于 artifacts/rmul_2027_3v3。

CAD 使用毫米、Y 为竖直轴。转换为世界坐标 (X,Z,-Y)/1000。CAD 场内边界测量约 12×8 米；外轮廓约 12.6×8.6 米。尺寸来自官方 CAD，未替代现场及规则手册逐项核验。

导航使用原 Point-LIO、small_gicp、ThetaStar 和 OmniPid。2027 二维建图使用原机器人已有水平雷达；MID360 的里程计、重定位和地形障碍输入保留。未注入仿真真值作为定位输入。

TF 已记录：map → odom → base_footprint，以及 base_footprint → front_mid360。map → odom 的实际输出为 {transforms['map->odom']['translation']}，不是配置中的零初值。

## 已解决的问题

1. STEP 轴向与单位：完成旋转及毫米到米转换，保留高差、孔洞和静态凹网格碰撞。
2. 原启动入口限定旧场地：新增独立 2027 入口，为原机器人生成机制增加可选配置路径，默认旧配置保持原样。
3. 初始二维图覆盖不足：使用已有水平雷达建图；目标选择按实际矩形足迹检查新增扫掠区域。
4. 地图灰色未知像素被加载为空闲：保存/加载阈值固定为 free_thresh=0.196；全局跟踪未知区，ThetaStar 禁止在未知区规划。
5. Point-LIO 初始化漂移：实测下落阶段 IMU 加速度接近零；增加落地、静止与 IMU 稳定检查后再启动定位。原 Point-LIO 下采样参数已经恢复，最终未替换算法。
6. 进程与资源：共用启动锁、检查单实例并清理本次启动树；原始 PCD 归档到产物目录。
7. RViz 黑屏：重启 WSLg 后曾恢复并获用户确认，重开窗口后又复发，已再次重启会话。{ui_followup} 原 ready-final 通过记录保留，手动运动期间的静止检查不作为定位漂移验收。

先前的失败、默认阈值测试及稠密参数诊断记录保留，均不作为最终通过依据。

## 手动扩展测试暴露的问题

原框架线速度上限 2.5 m/s，本轮限速 0.5 m/s。卡墙时仿真实时倍率约 0.323，按墙钟观察的速度上限约 0.162 m/s。此次未提高速度。

保存地图约 21% 栅格已知。手动目标将车带至地图未知区，真值世界位置约 (1.211,-2.239)，位于东侧 0.20 米高平台边缘；当时定位误差约 0.173 米，small_gicp 连续不收敛，规划与恢复失败。该记录证明当前局部导航的覆盖限制，不能据附近简单目标成功推断全场可用。

已保存 manual-wall/runtime.json、导航事件和原日志索引，并重启回出生点。field-reference-ready 检查通过：静止漂移约 0.013 米，真值定位误差约 0.010 米。

RViz 新增完整 CAD 参考图层，消息、尺度、资源路径及坐标检查通过。Gazebo 继续无界面运行，RViz 刷新限制为 5 FPS。参考模型仅用于显示；未生成全场导航地图、未扩大定位先验，也未解决全场避障问题。完整场地与已观测地图见 full-field-vs-map.png。

## 2026-10-19 路径显示跟进

原行为树与限速不变。RViz 新增独立粉色全局路径图层，抬高显示并调整全局/局部线宽；用户已确认能看到通向目标的粉色规划路径。短目标返回 SUCCEEDED，真值移动约 0.680 米、终点误差约 0.110 米，约 10.06 秒；全局路径更新 21 次，局部路径更新 147 次。

本轮最高速度指令约 0.393 m/s，仿真实时倍率约 0.393，观感慢符合当前参数与计算负载。完整数值和原始记录见 route-display-nav-20261019；说明见 NAVIGATION_DISPLAY.zh-CN.md。

## 当前边界与后续

地图仅覆盖传感器已观测区域，未知区保留。未验收整场建图、坡道通过、狭窄通道、动态绕障、比赛战术或 Venom 实车。占用目标测试证明规划拒绝，不等于动态绕障通过。

原 Venom 保持原状。未创建 Git 提交、未推送。新场景导航/仿真分支均为 sim/rmul2027-3v3；旧兼容补丁与上游历史、许可证、原 README 保留。

[官方场地来源](https://bbs.robomaster.com/article/1970956?source=4)。源文件 SHA256：7d47c1b6a6a0253a732e46cb122bb84410b11d5134bcb5658104299db4a48ca4。

{ui_followup} 后续应先补全定位先验与导航地图，检查未知区及平台侧边的规划、控制行为，再扩展目标范围和速度。完整逐终端命令和黑屏恢复步骤见 README.zh-CN.md。
'''
(folder/'TEST_RESULTS.zh-CN.md').write_text(text)
(artifact/'TEST_RESULTS.zh-CN.md').write_text(text)
print(json.dumps({'nav_error_m':nav['ground_truth_goal_error'],'map_cells':cells,
                  'pcd_points':points,'report':str(folder/'TEST_RESULTS.zh-CN.md')},ensure_ascii=False))
