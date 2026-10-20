# 北极熊框架 × RMUL 2027 3V3 基础导航验证

本工程只验证北极熊原有定位、规划和控制链能否在2027场景中运行。地图来源为官方RMUL2027_3V3_V1.0.0.stp。没有进行Venom实车适配或完整比赛战术开发。原始导航算法没有重写；依赖增加了可关闭诊断、环境兼容及可选仿真控制参数。

## 当前证据

C-010诊断开启的原圆柱候选已通过两次完整路线。C-011正在验证诊断关闭、原CAD对照和独立检出；最终结果见TEST_RESULTS.zh-CN.md。运行成功不能直接写成未经修改的上游仓库原样通过。

两种场景必须分开记录：cad保留官方CAD转换后的原碰撞网格；seam-ramps增加12段低接缝过渡斜面，是辅助导航近似。两者使用相同2027 CAD派生地图，高平台和孔洞保守视为障碍，不代表三维通行层或隧道通过能力。

## 获取与恢复

需要Ubuntu22.04 amd64、ROS2 Humble、Gazebo Fortress，以及scripts/venom/environment.lock.tsv对应的已安装依赖。FreeCAD仅在重新转换STEP时需要，直接运行附带网格和地图不需要FreeCAD。

在一个新的WSL工作区中克隆本分支到src/pb2025_sentry_nav，进入该仓库执行：

```bash
python3 scripts/rmul2027/prepare_rmul2027.py
bash scripts/rmul2027/build_rmul2027.sh
```

恢复工具按现有dependencies.lock.repos与子模块SHA获取源码，验证patches/manifest.json并应用三个依赖补丁。重复执行会识别已应用补丁；版本不符、部分补丁或资源冲突时停止，保留现场。已有工作区可用--source-cache提供固定Git对象缓存；缓存不会作为运行时overlay。--legacy-pcd可导入校验通过的原2025大点云，2027基础运行不需要它。

assets包含约2MB导航地图、必要高度缓存、STEP来源清单，以及97KB官方TF2 deb。prepare仅校验并解包TF2到工作区artifacts/rmul_2027_3v3/runtime/tf2-0.25.24，不执行系统安装；保留系统旧TF2。包来源、SHA和上游修复见assets/external-resources.json。原2025的121MB PCD需按其官方分享地址获取，不纳入Git。

## 运行与观察

终端A，在本仓库执行。默认cad，不自动发目标，不打开Gazebo图形窗口：

```bash
bash scripts/rmul2027/start_rmul2027_test.sh
```

辅助近似场景必须显式选择：

```bash
bash scripts/rmul2027/start_rmul2027_test.sh seam-ramps
```

两者固定原圆柱轮碰撞体、诊断关闭、IMU队列5和项目局部TF2库。相机640×360@5、IMU200Hz、原PI及时间单位保持当前验证配置。缺地图、修复库或构建产物会报错，不回退其他场景或旧库。底层start_rmul2027.sh仍保留旧默认和显式参数，用于历史对照；日常本轮验收使用带_test的入口。

终端B，在本仓库运行轻量RViz：

```bash
bash scripts/rmul2027/start_rviz.sh
```

使用Nav2 Goal在可通行地面按住左键并拖动朝向。完整CAD图层仅供视觉参照，导航使用2027占用地图与实时局部代价地图。保持一个RViz，自动审计期间不手动发目标。

停止终端A可按Ctrl+C；另一个终端可运行：

```bash
python3 scripts/rmul2027/stop_rmul2027.py
```

清理只处理该项目启动树，不全局杀ROS或重启WSL。

## 自动验收

先停止已有仿真。在工作区source Humble和install/setup.bash后，从仓库执行以下命令；record-dir必须是不存在的新目录。此命令会主动移动机器人，结束会关闭自己启动的仿真、观察器和录包：

```bash
python3 scripts/rmul2027/run_timing_experiment.py \
  --record-dir /tmp/rmul2027-new-test \
  --params-file scripts/rmul2027/config/nav2_test.yaml \
  --gz-world-path scripts/rmul2027/config/gz_world_test.yaml \
  --field-contact cad --last-case 07-occupied
```

大型记录请将record-dir改到D盘WSL工作区的artifacts。seam-ramps对照只改变--field-contact。完整路线包括静止、近点、右上、中央、左下、左上、重复右上和占用目标拒绝。任一检查失败立即停止后续目标；suite-summary.json的completed与逐项passed决定结果，不能把前缀通过写成整套通过。

运动目标要求180墙钟秒内SUCCEEDED、真值发生位移、估计和真值终点误差<0.35m、底盘停车及实际Gazebo姿态合理。保留时间对齐误差保护、TF和双图足迹新鲜度、核心节点active且单实例。占用目标须ABORTED、无非零运动指令。真值仅用于验收，不输入定位。

每次保留实际参数、机器人SDF、world、库hash、地图与配置hash、自有goal UUID、trace、代价地图和录包。失败保护点单独保存guard-trigger.json，失败时另存failure-model-pose.txt。render_baseline_audit.py生成误差曲线，数据空档不连线。

## 修改边界与已知限制

TF2采用官方0.25.24核心库解决已在本机取证的双线程互锁，不修改系统包。Point-LIO仅增加默认关闭的接收诊断和默认5条的可配置队列。仿真生成入口支持已有机器人碰撞／相机／PI覆盖；底盘插件保留原缺省值，2027才显式选用PI配置。补丁还保留裁判shebang与LightBar兼容修正。具体可审查patches中的三个完整diff。

合成点时间和timestamp_unit=2存在已知输入契约问题，本轮未修；诊断关闭时没有内部时序报告，不能将空报告视为无漏收。若再次定位跳变，保留失败结论，不继续叠加队列、滤波、速度或地形修改。

historical说明、早期报告生成器、before配置和被拒绝候选仅为历史资料，不用来生成本轮验收结论。C盘唯一协作交接read.md仍由当前开发者维护；本仓库文档是可发布的工程说明与结果快照。
