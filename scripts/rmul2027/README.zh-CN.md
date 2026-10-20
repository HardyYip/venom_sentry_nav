# 北极熊框架 × RMUL 2027 3V3 基础导航验证

本工程只验证北极熊原有定位、规划和控制链能否在2027场景中运行。地图来源为官方RMUL2027_3V3_V1.0.0.stp。没有进行Venom实车适配或完整比赛战术开发。原始导航算法没有重写；依赖增加了可关闭诊断、环境兼容及可选仿真控制参数。

## 当前证据

C-015完成精确失败点对照：原cad-flat-marking／software复现无物理进展，新cad-flat-floor只将两处3毫米地板碰撞接平，视觉和导航地图保持原样。软件、GPU同两目标各3/3通过，两次全新GPU完整路线各8/8；候选16个运动目标成功停车、2次占用拒绝。最大真值终点误差0.191830米。0.35米保护和180秒限时不变。结论依赖既有兼容补丁和列明碰撞近似；官方真实铺设尚未确认。详见TEST_RESULTS.zh-CN.md和reports/C015-summary.json。

| 场景 | 含义 |
| --- | --- |
| cad-flat-floor | 新默认：中央标识无碰撞，加两处3mm地板碰撞接平，属于仿真接触近似 |
| cad-flat-marking | C-013候选：只移除中央标识碰撞；C-015精确目标仍复现地板拼接阻塞 |
| cad | 全部原CAD碰撞，保留失败复现 |
| seam-ramps | 历史12段过渡斜面近似，不作当前默认 |

高台、墙体及孔洞仍保守阻挡，未开启完整三维通行或隧道能力。C-011／C-013结果保留在history和reports。

## 获取与恢复

需要Ubuntu22.04 amd64、ROS2 Humble、Gazebo Fortress，以及scripts/venom/environment.lock.tsv对应的已安装依赖。硬件渲染探测另需mesa-utils（glxinfo）；auto在无法确认硬件加速时明确提示并回退software，显式gpu则报错停止。Python依赖包括xmacro==1.2.1、numpy、scipy、matplotlib、PyYAML和Pillow；已有环境无需重复安装。FreeCAD仅在重新转换STEP时需要，直接运行附带网格和地图不需要FreeCAD。

在一个新的WSL工作区中克隆本分支到src/pb2025_sentry_nav，进入该仓库执行：

```bash
python3 scripts/rmul2027/prepare_rmul2027.py
bash scripts/rmul2027/build_rmul2027.sh
```

构建入口会在自身进程中排除Windows盘上的PATH条目，避免Anaconda的Protobuf污染Linux Gazebo构建；不修改全局环境。恢复工具通过单次Git命令HTTP/1.1获取依赖，按现有dependencies.lock.repos与子模块SHA获取源码，验证patches/manifest.json并应用三个依赖补丁。重复执行会识别已应用补丁；版本不符、部分补丁或资源冲突时停止，保留现场。已有工作区可用--source-cache提供固定Git对象缓存；缓存不会作为运行时overlay。--legacy-pcd可导入校验通过的原2025大点云，2027基础运行不需要它。

assets包含约2MB导航地图、必要高度缓存、STEP来源清单，以及97KB官方TF2 deb。prepare仅校验并解包TF2到工作区artifacts/rmul_2027_3v3/runtime/tf2-0.25.24，不执行系统安装；保留系统旧TF2。包来源、SHA和上游修复见assets/external-resources.json。原2025的121MB PCD需按其官方分享地址获取，不纳入Git。

候选场景资产已包含，无需接手者安装FreeCAD。需要重建本次中央碰撞处理时执行：

```bash
python3 scripts/rmul2027/build_flat_marking_scene.py
```

新默认地板接平另执行 python3 scripts/rmul2027/build_flat_floor_scene.py。它锁定原mesh SHA、82个顶点／270个相关面／16个退化侧面；4000采样点原有支撑均保留，不直接删去地板。其他3326个记录逐字节相同，原z0地面266面保留。

该工具只依赖Python标准库，锁定原mesh SHA且必须恰好匹配60面，否则在写输出前停止；保留其余3596面原始字节，中央3600个5cm地面采样点均有原z0支撑。不批量删除其他薄层，不添加斜面。

## 运行与观察

终端A，在本仓库执行。默认cad-flat-floor／auto渲染，不自动发目标，不打开Gazebo图形窗口：

```bash
bash scripts/rmul2027/start_rmul2027_test.sh
```

原始CAD碰撞对照必须显式选择：

```bash
bash scripts/rmul2027/start_rmul2027_test.sh cad
```

历史辅助近似场景必须显式选择：

```bash
bash scripts/rmul2027/start_rmul2027_test.sh seam-ramps
```

第二个参数可选auto／gpu／software，例如 bash scripts/rmul2027/start_rmul2027_test.sh cad-flat-floor software 可恢复软件渲染。auto先检测硬件OpenGL，WSLg可选择D3D12；实际选择写入rendering-runtime.json。传感器配置及物理步长不变。

这些选项固定原圆柱轮碰撞体、诊断关闭、IMU队列5和项目局部TF2库。相机640×360@5、IMU200Hz，2027仿真显式使用P=(1000,1000)、I=(200,200)，时间单位保持当前验证配置；底盘插件的上游缺省P=(100,500)、I=0仍保留，不能把此候选描述为上游零改动。缺地图、修复库或构建产物会报错，不回退其他场景或旧库。底层start_rmul2027.sh仍保留旧默认和显式参数，用于历史对照；日常本轮验收使用带_test的入口。

终端B，在本仓库运行轻量RViz：

```bash
bash scripts/rmul2027/start_rviz.sh
```

RViz也支持auto／gpu／software，默认auto，本机实测D3D12 RTX5070；其他电脑需核对实际日志。

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
  --field-contact cad-flat-floor --render-mode gpu --last-case 07-occupied
```

大型记录请将record-dir改到D盘WSL工作区的artifacts。seam-ramps对照只改变--field-contact。完整路线包括静止、近点、右上、中央、左下、左上、重复右上和占用目标拒绝。任一检查失败立即停止后续目标；suite-summary.json的completed与逐项passed决定结果，不能把前缀通过写成整套通过。

C-014精确目标使用 --route-set manual-c014，依次为(9.675,-5.375)与(-0.075,-0.625)。原对照显式选cad-flat-marking／software。

中央标识专项使用同一命令，将--last-case 07-occupied替换为--route-set central-marking。默认专项依次为静止、(4.20,-5.71)、(9.50,-5.71)、(5.53,-4.85)、(5.53,-4.15)、(5.53,-4.85)。原CAD对照只改--field-contact cad，并可用--last-case 02-manual-second截断。录包现包含完整代价地图／updates及隐藏action状态／feedback。

运动目标要求180墙钟秒内SUCCEEDED、真值发生位移、估计和真值终点误差<0.35m、底盘停车及实际Gazebo姿态合理。保留时间对齐误差保护、TF和双图足迹新鲜度、核心节点active且单实例。占用目标须ABORTED、无非零运动指令。真值仅用于验收，不输入定位。

每次保留实际参数、机器人SDF、world、库hash、地图与配置hash、自有goal UUID、trace、代价地图和录包。失败保护点单独保存guard-trigger.json，失败时另存failure-model-pose.txt。render_baseline_audit.py生成误差曲线，数据空档不连线。

## 修改边界与已知限制

TF2采用官方0.25.24核心库解决已在本机取证的双线程互锁，不修改系统包。Point-LIO仅增加默认关闭的接收诊断和默认5条的可配置队列。仿真生成入口支持已有机器人碰撞／相机／PI覆盖；底盘插件保留原缺省值，2027才显式选用PI配置。补丁还保留裁判shebang与LightBar兼容修正。具体可审查patches中的三个完整diff。

WSLg的WARN:COPY MODE／无窗口问题只曾通过显示服务恢复，永久修复未做；不要为此重复启动仿真。其他拐角与全场任意目标未充分验证。

合成点时间和timestamp_unit=2存在已知输入契约问题，本轮未修；诊断关闭时没有内部时序报告，不能将空报告视为无漏收。若再次定位跳变，保留失败结论，不继续叠加队列、滤波、速度或地形修改。

historical说明、早期报告生成器、before配置和被拒绝候选仅为历史资料，不用来生成本轮验收结论。C盘唯一协作交接read.md仍由当前开发者维护；本仓库文档是可发布的工程说明与结果快照。
