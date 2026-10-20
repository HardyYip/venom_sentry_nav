# RMUL 2027 3V3 × 北极熊导航仿真

> 当前验证状态：2026-10-19晚间固定路线验收发现运动中定位跳变，尚未完成验收。此前四条成功路线仅为历史记录；最新问题与进度见TEST_RESULTS.zh-CN.md顶部及唯一交接read.md。没有应用TF2升级。


更新：2026-10-19（Asia/Shanghai）。当前默认使用完整 CAD 导航地图、仿真滚轮接触配置及低台阶窄斜面过渡的导航场景。原 CAD 场景独立保留。原传感器局部地图、原场景及失败记录保留。当前结果和限制见 TEST_RESULTS.zh-CN.md。

## 位置与启动

工作区 /home/hardy/pb2025_nav_ws 位于 D 盘 WSL 虚拟磁盘。ROS 2 Humble，Gazebo Fortress 6.18.0，命名空间 red_standard_robot1。代码位于 src/pb2025_sentry_nav/scripts/rmul2027；STEP、地图与实验记录位于 artifacts/rmul_2027_3v3。日志位于 logs/时间-rmul2027-nav。

若后台仿真与单个 RViz 已打开，直接使用，不重复启动。重新启动时使用两个 Ubuntu / WSL 终端。

终端 A：启动无界面的 Gazebo、机器人与定位导航，等待落地和 IMU 稳定后才启动定位，不自动发送目标。

```bash
bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/start_rmul2027.sh nav
```

默认等价于 nav --map-set cad --wheel-contact rounded --field-contact seam-ramps。与旧启动器共用锁，提示 Another simulation is running 时先检查已有进程。

终端 B：只打开一个轻量 RViz。Gazebo 保持服务器模式，不打开第二个图形窗口。

```bash
bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/start_rviz.sh
```

在 RViz 点击 Nav2 Goal，在可通行的白色区域按住左键并拖动朝向。顶层“到目标的规划路径”是粉色全局路径，订阅 /red_standard_robot1/plan；蓝色线是控制器局部路径。“Official CAD Reference”是完整场地三维参考图层，可单独开关。当前真正供规划使用的完整栅格也来自官方 CAD，不再只覆盖出生点附近。

地图上墙、平台、孔洞等为占用区域；高代价格是障碍物附近的缓冲区。目标应给机器人整个足迹留出空间。墙内目标会在重试一次规划后失败，不会为无效目标执行退让。执行路径时发生控制失败仍可清理局部地图并退让 0.2 米，最多两次。

停止本项目后台仿真：

```bash
python3 /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/stop_rmul2027.py
```

它检查 PID 与命令，只停止此项目拥有的启动树。可以保留当前 RViz 进程，重启仿真后会重新接收数据。不要并行启动旧场景和 2027 场景。

## 当前地图与物理配置

maps-cad/rmul_2027_3v3.yaml、PGM、PCD 由官方碰撞网格生成；并非传感器 SLAM 建图。分辨率 0.05 米，254×174 栅格，31,166 空闲、13,030 占用格。高台、坡道及缺地位置保守设为障碍，不启用登台策略。PCD 152,326 点。完整覆盖表示 CAD 已知静态场地，不表示真实环境或移动物体均已观测。

CAD 使用毫米，世界坐标=(X,Z,-Y)/1000。世界到 map 平移=(4.8,-3,0)。PCD 保存时逆变换到原 small_gicp 加载器期待的雷达坐标，加载器再施加一次标定变换，避免重复旋转。仿真真值只用于评估，不输入定位算法。

Point-LIO、small_gicp、ThetaStar、OmniPid 沿用原框架。2027 关闭默认自旋，线速度上限 0.5 m/s，接近目标会减速。二维 footprint 为 0.58×0.50 米矩形，加 0.02 米 padding。

rounded 为可选的导航仿真接触近似：沿用原机器人质量、惯性、轮心、轮径、宽度和外观，将四个完整圆柱碰撞轮各替换为 32 个球形滚轮包络。它用于避免横移时完整圆柱侧缘卡在 CAD 低台阶；不是经过标定的真实麦克纳姆轮动力学模型。原 CAD 碰撞场景完整保留中央约 10–11 毫米和右侧约 3 毫米的地面高差。但直接长路线仍可能在中央竖直边缘横移卡住，因此默认导航场景从 CAD 高度缓存自动提取 12 段 3–11 毫米低接缝，为它们添加水平长度 6 厘米的斜面过渡，另有低侧 1.5 厘米提前量及高侧 2.5 厘米重叠，顶面比原高度高 0.1 毫米；不降低高差、不覆盖孔洞、不改变高台或墙。过渡是明确的碰撞近似，不是官方 CAD 原貌。源脚本 build_navigation_contact_scene.py、192 三角形网格及 scene/navigation-contact-manifest.json 可复查。仅 scene 模式默认查看原 CAD 碰撞场景。

2027 的 MecanumDrive2 速度环使用 P=(1000,1000)、I=(200,200)，积分输出限幅=(100,200) N、轴向力限幅=(200,400) N；收到对应轴零速指令会清除积分。缺省值仍为原配置 P=(100,500)、I=0、力限幅=(100,200) N，旧场景不启用新参数。仿真工业相机由 1920×1080@30 降为 640×360@5；MID360、IMU 与定位下采样参数保留。

早先实时倍率约 0.32–0.39，本次轻量配置测试约 0.8。0.5 m/s 是仿真时间中的速度上限；按现实时间观察还受到实时倍率与接近目标减速影响。当前限速优先用于基础导航验收。

复现原 CAD 场地、原机器人接触与旧局部地图，可显式选择：

```bash
bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/start_rmul2027.sh nav --map-set sensor --wheel-contact original --field-contact cad
```

该组合保留了局部覆盖限制及低台阶卡滞问题。使用完整地图检查原 CAD 碰撞，可选 nav --map-set cad --wheel-contact rounded --field-contact cad；这也仍有中央横移卡滞记录，不能作为已通过的默认导航配置。

## 复现导航验证

另开 Ubuntu 终端，确认没有手动目标正在执行。下面 nav 命令会发目标并移动机器人；ready 只做静止检查。

```bash
source /opt/ros/humble/setup.bash
source /home/hardy/pb2025_nav_ws/install/setup.bash
python3 /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/verify_simulation.py --mode nav --goal-x 9.3 --goal-y -0.2 --goal-timeout 180 --stability-seconds 3 --record-dir /home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/manual-far
```

该远点使用 map 坐标，适合从初始出生点验证。验收包括 action 成功、真值位移、终点误差≤0.35 米、底盘停车、节点 active 且单实例、传感器和 TF，并保留轨迹、路径、实际代价地图和物理姿态。测试发现定位误差超过 0.35 米，或有运动指令却连续 8 秒仿真时间不动时会取消目标；这仅是自动验收保护，不替代手动导航恢复机制。

占用目标拒绝测试会自动选择附近已知占用格，并通过 NavigateToPose 发送；预期 ABORTED、无非零速度指令、真值位移<0.05 米。

```bash
python3 /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/verify_simulation.py --mode nav --scenario invalid --stability-seconds 3 --record-dir /home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/manual-invalid
python3 /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/verify_simulation.py --mode ready --stability-seconds 12 --record-dir /home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/manual-ready
```

静止检查期间不要手动发送目标。blocked 场景仅调用 ComputePathToPose；invalid 才会验证完整导航行为树对占用目标的拒绝。

## 固定路线验收入口

先启动一套2027默认nav并确认没有手动目标；运行期间不要同时点RViz目标。这个脚本会移动机器人，依次检查静止、附近短目标、右上、中央、左下、左上、重复右上和占用目标拒绝。任何检查失败就停止后续目标，保留原始记录；没有自动升级或修改导航参数。

终端C执行，record-dir必须使用新的目录：

```bash
source /opt/ros/humble/setup.bash
source /home/hardy/pb2025_nav_ws/install/setup.bash
python3 /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/run_baseline_audit.py --record-dir /home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/audit-new-001
```

入口自动核对唯一运行的2027日志；保留参数快照、进程／目标、逐段结果和本轮日志区间。verify_simulation.py会保存自有目标UUID，清理只对该UUID发取消请求。套件自带只读观察器，最长20分钟后退出；脚本结束不自动关闭Gazebo或RViz。

完成或失败后生成轨迹和时间对齐定位误差图：

```bash
python3 /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/render_baseline_audit.py --record-dir /home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/audit-new-001
```

检查suite-summary.json的completed和每项passed，不能把只通过前几段当作整套通过。原历史验收生成器generate_report.py含对持续日志的旧断言，不用于覆盖本轮结果。

## 转换、建图与构建

官方来源：https://bbs.robomaster.com/article/1970956?source=4 。STEP 文件 1,989,786 字节；SHA256=7d47c1b6a6a0253a732e46cb122bb84410b11d5134bcb5658104299db4a48ca4。

场地约 12×8 米，CAD 外轮廓约 12.6×8.6 米。FreeCAD 将原融合实体与外圈 shell 分件导出；可视与碰撞网格分别保存，各约 3811 三角形，内部 world 名仍为 default。

```bash
python3 /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/convert_field.py --step /home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/source/RMUL2027_3V3_V1.0.0.stp
python3 /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/build_cad_navigation_maps.py
python3 /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/build_navigation_contact_scene.py
```

重编底盘插件必须在 D 盘 WSL 工作区执行：

```bash
cd /home/hardy/pb2025_nav_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rmoss_gz_plugins --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

仅看场景使用 start_rmul2027.sh scene。传感器建图使用 start_rmul2027.sh mapping 与 save_mapping.py；其输出保存到 maps，不覆盖 maps-cad。使用新传感器地图时须显式选择 nav --map-set sensor。原传感器地图、Point-LIO 原始文件和历史实验均保留。

## 显示异常与验收边界

WSLg 曾在 RViz 重开后黑屏；用户此前已确认完整场地和粉色规划路径可见。保持单个 RViz，避免反复关开窗口。显示故障时先检查窗口和 weston 日志，不能以 RViz 进程存在判断画面正常。上游同类问题：https://github.com/microsoft/wslg/issues/1456 。停止或重启显示服务需先保存其他 WSL 图形任务，常规导航无需重启整个 WSL。

保留上游许可证、作者归属、原 README、提交历史、两个原兼容补丁。导航与仿真仓库在 sim/rmul2027-3v3；依赖仓库保持原固定版本，底盘插件有未提交的可选参数补丁。未提交、未推送。Venom 旧工作区与实车接口保持原状。

当前结果属于带窄斜面碰撞近似的静态导航场景，不能宣称原 CAD 竖直台阶的真实通过能力。验证内容为代表性远点、保留高差的过渡以及不可达目标拒绝。原 CAD 和导航近似的差别可看 contact-approximation.png。未验收所有目标点、平台坡道、动态比赛道具、真实麦克纳姆轮动力学或 Venom 实车。旧 Venom 仿真使用 TEB，本轮未做相同场景和限速下的公平性能对比，不能据此给框架排名。

## TF2 冻结修复的可选验证入口

本机系统 TF2 0.25.23 已在实际 Nav2 中复现双线程互锁，会使代价地图足迹时间戳停住。项目保留官方 0.25.24 核心库的局部副本，不安装或替换系统 ROS 包。当前仅做可撤销验证，默认启动命令仍使用系统库；最新结果以唯一交接 read.md 为准。

停止已有本项目启动树后，在 Ubuntu 终端使用：

```bash
bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/start_rmul2027.sh nav --tf2-library-dir /home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/runtime/tf2-0.25.24/lib
```

选项只影响导航子进程，Gazebo 与机器人生成保留原环境；运行日志 tf2-runtime.json 记录选定库与 SHA256。去掉该选项后重新启动即可恢复系统库。只支持 nav 模式。此修复针对 TF2 死锁，不能代替运动定位误差或轮胎接触问题的验证。上游修复：https://github.com/ros2/geometry2/pull/990 。

## Point-LIO 接收诊断与时间单位对照（2026-10-20）

Point-LIO新增可选参数timing_diagnostics（默认false）及imu_queue_depth（默认5），均在节点启动时读取，需要重启生效。诊断每秒记录内部IMU时间间隔、疑似漏收、回调数据年龄、点时间范围和计算耗时；疑似漏收必须与外部原始录包对照，不能单看计数定因。

本轮原参数与秒单位两组都未发现内部IMU漏收；原参数远目标因0.531m误差停止，秒单位候选短目标因0.917m误差停止，因此队列400按条件未运行、不采纳；秒单位候选未成为默认。秒单位下发现20次里程计时间戳回退，人工0.1s点时间范围与部分0.05s帧间隔重叠，需另行调查。默认转换器、轮胎、场地和IMU设置未改。

run_timing_experiment.py --record-dir <新目录> --params-file <候选YAML>复用既有审计，默认只运行到03-center；失败即停止后续目标并正常收尾自有目标／录包／进程。它会移动仿真机器人，执行前必须停止已有本项目运行树。使用前source Humble与本工作区。analyze_timing_experiment.py --record-dir <实验目录>在录包收尾后匹配外部IMU和内部接收时间戳，生成reception-analysis.json与对照图。

误差保护现立即保存guard-trigger.json与完整trace，绘图包含触发点且不跨超过0.25s的数据空档连线。0.35m保护未放宽。旧rmuc2025就绪＋近点回归通过；当前恢复原2027参数和显式TF2局部修复库，静止检查通过，运动跳变仍未解决。完整报告与当前运行现场见唯一交接read.md的C-009记录。

## C-010 轮胎接触对照与完整验收（2026-10-20）

球体控制复测远点触发0.473449m定位误差保护；圆柱轮候选先通过近点／远点／中央，再从两次全新启动完成每轮8项完整路线，12个运动目标全部SUCCEEDED并停车，2次占用目标拒绝，无backup／空局部路径。最大真值终点误差0.164847m，采样时间对齐误差峰值0.263557m，0.35m门槛未改。证据不代表全部场地或Venom实车验收。

当前默认仍是rounded，未采用候选。对照使用相同参数、world、PI、传感器与出生点；实际SDF去掉轮胎collision后完全相同。圆柱候选仍启用timing_diagnostics=true、队列5、原timestamp_unit=2；点时间契约问题保留，关闭诊断的生产配置尚未单独验证。少量IMU漏收没有与全部失败对应，未叠加队列400。

证据根目录为/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/diagnostics/contact-ab-20261020-125941。该目录REPORT.zh-CN.md含候选启动、完整实验复现及恢复命令，summary.json含逐段结果与接收／接触指标。唯一交接read.md为接手依据。

run_timing_experiment.py新增可选--gz-world-path并记录配置SHA，未指定时行为保持原样。analyze_contact_experiment.py在录包正常收尾后按实际阶段解析IMU、平面真值和raw LIO；不从真值Odometry的零Z／零横滚俯仰推断三维姿态，不将相邻raw LIO位移直接当作同帧定位误差。仅离线分析，不增加导航输入。
