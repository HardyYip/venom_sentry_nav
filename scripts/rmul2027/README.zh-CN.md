# 北极熊 × RMUL 2027 3V3 基础导航仿真

目标是验证北极熊机器人在2027场地中的基础定位、规划、控制，以及规定的固定障碍和单个同速对向移动物体避让。场地来源为官方RMUL2027_3V3_V1.0.0.stp；不扩展到Venom实车、多车或比赛战术。

C024/C025补齐独立下游停车、Nav2位姿时效门控和Point-LIO点时间链路；两次全新快速完整路线及普通完整路线、静态绕障、封路停车/移除后恢复、2025旧图近点回归已通过。普通与fast入口采用已验证的instantaneous秒单位快照、IMU队列400及真实IMU缺口锁止。定位基线见[C025报告](reports/C025-RESULTS.zh-CN.md)；历史C022未采用快照的失败仍保留，不能混为当前结果。

当前普通0.5m/s入口采用运动自旋3.14rad/s、零Nav2输入停止追加自旋，以及真实水平RPLidar的源时间戳车体footprint过滤和独立2秒观测层。C038同一配置固定四类各3次12/12、单个0.5m/s对向代理3/3、普通完整路线8/8、停放移除清除和封路恢复通过，固定/移动真实接触均0；到点与安全停车均通过原1秒全零阈值。结果和边界见[C038报告](reports/C038-RESULTS.zh-CN.md)。C034原1秒停车失败、C035无自旋危险逼近、C036规划失败保留；不把后续有限通过写成唯一根因已证实。 C038仅将接近最低速度0.1降为0.05m/s，额定速度保持；C037原1秒停车失败另见reports/C037-RESULTS.zh-CN.md。

## 队友获取、恢复与构建

本次验证环境：Ubuntu22.04 amd64、ROS2 Humble、Gazebo Fortress6.18。先具备ROS／Gazebo及scripts/venom/environment.lock.tsv所列依赖。Python需要PyYAML、numpy、scipy、matplotlib、Pillow、xmacro==1.2.1，GPU探测需要mesa-utils。无需安装FreeCAD即可使用附带网格与地图。

在新的Linux／WSL工作区执行：

```bash
mkdir -p ~/rmul2027_ws/src
cd ~/rmul2027_ws
git clone --branch sim/rmul2027-3v3 https://github.com/HardyYip/venom_sentry_nav.git src/pb2025_sentry_nav
cd src/pb2025_sentry_nav
source /opt/ros/humble/setup.bash
python3 scripts/rmul2027/prepare_rmul2027.py
rosdep install --from-paths ../../src --ignore-src --rosdistro humble -y
bash scripts/rmul2027/build_rmul2027.sh
```

prepare按已发布依赖基准与patches/manifest.json恢复源码及完整补丁，校验资源并只在项目内解包TF2。重复运行识别已应用状态，遇到已有冲突停止，不覆盖用户修改。--source-cache可提供固定Git对象缓存，但不得作为运行overlay；2027不依赖原2025的121MB点云。做旧地图回归才需要按原文档获取该点云，并用--legacy-pcd导入校验。

## 启动与手动测试

普通入口运动时自旋，零Nav2输入时不再追加自旋；到点与安全停止均要求vx/vy/w归零。普通自动验收默认严格全零，显式`--spin-mode`仅供历史到点平移停止对照。fast入口保持C025配置，本轮新避障结论只覆盖普通0.5m/s。

终端A，在仓库目录执行。普通入口保持0.5m/s；只运行一套后台Gazebo，不发目标、不打开Gazebo GUI：

```bash
bash scripts/rmul2027/start_rmul2027_test.sh
```

需要已验证的2.5m/s上限、提前2m减速配置时改用：

```bash
bash scripts/rmul2027/start_rmul2027_fast.sh
```

两入口与RViz共用项目局部FastDDS64MiB传输，系统ROS／全局DDS不变。底层start_rmul2027.sh保留历史默认，不能与这些入口混用当作同一验收配置。RMUL2027_DDS_PROFILE可显式覆盖，但其他机器须核对点云体积、频率、传输及保护状态，不能照抄64MiB当通用标定。

终端B，在同一仓库运行单个轻量RViz：

```bash
bash scripts/rmul2027/start_rviz.sh
```

用Nav2 Goal在已知可通行空地按住左键，拖动朝向后松开。先近点，再隔着障碍物换目标，检查粉色规划线从当前车位置开始，实际运动并停车。CAD完整图层只供视觉参照，导航使用占用地图＋实时全局／局部代价图。自动测试时不要手动发目标。

两导航入口支持[场景] [auto|gpu|software]，RViz支持[auto|gpu|software]。默认auto先探测硬件加速；本机验证D3D12 RTX5070。显式gpu不能确认硬件就报错，software是明确选择的CPU回退，物理／ROS算法仍需CPU。无需同时开Gazebo GUI。

停止本项目启动树：

```bash
python3 scripts/rmul2027/stop_rmul2027.py
```

关闭RViz后可以重开，单实例锁防止重复启动。WSLg下入口会以已验证的后台方式调用`restore_rviz_window.ps1`，30秒内查找并恢复本项目的最小化／屏外窗口。`RMUL2027_RESTORE_WINDOW=0`可关闭这一步。C023独立检查三次串行开关均通过，没有重新驱动机器人；见[核查报告](reports/C023-AUDIT.zh-CN.md)。

若要从Windows PowerShell手动恢复，下面是本机示例；组员应替换发行版、用户名和工作区。`-File`是Windows能访问的脚本路径，`-ConfigTitle`必须是RViz标题中的Linux配置路径，两者不同：

```powershell
$rvizTools = '\\wsl.localhost\Ubuntu-22.04\home\hardy\pb2025_nav_ws\src\pb2025_sentry_nav\scripts\rmul2027'
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$rvizTools\restore_rviz_window.ps1" -ConfigTitle '/home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/config/interactive-light.rviz'
```

当前后台组合（`nohup`、普通文件日志、关闭辅助进程的锁fd）已实测可用。此前改变了多个调用条件后才恢复，尚未分别证明哪一项单独造成`Invalid argument`，不把组合修正写成唯一根因。这只是显示层恢复；WSLg传输跨Windows／WSL重启的长期稳定性尚未证明，不通过重启导航处理窗口故障。

## 场景和未通过项

| 场景 | 含义／边界 |
| --- | --- |
| cad-flat-floor（默认） | 中央薄标识移除碰撞＋两处3mm地板碰撞接平；属于明确列出的仿真接触近似 |
| cad-flat-marking | 只移除中央薄标识碰撞；历史精确拐角仍有地板拼接阻塞 |
| cad | 保留全部原CAD碰撞，保留历史失败复现 |
| seam-ramps | 历史12段斜面过渡近似 |

原CAD、视觉、地图及历史失败均保留。不宣称官方确认薄层真实铺设方式；也没有开放高台、隧道或任意三维通行。

普通与fast验证入口使用instantaneous＋timestamp_unit=0＋point_time_is_valid=true；IMU队列400，真实IMU缺口超过0.1秒则锁止定位并要求重新初始化。快照批量更新的信息权重修正后，两次快速与一次普通完整路线通过，历史0.368m失败不会被改写。历史参数在history/before-C025-*；底层历史入口及原2025配置仍保持原选择。实车必须核对真实雷达逐点时间，不能照搬Gazebo整帧快照假设。

## 自动验收与证据

先停止已有仿真，source Humble与../../install/setup.bash。record-dir须不存在，每次只运行一组。先设置与实际机器相符的产物根目录；WSL必须位于Windows挂载分区，下面以E盘为例。原生Linux可改为有足够空间的普通目录。普通完整路线默认不录bag：

```bash
export RMUL_TEST_ARTIFACT_ROOT=/mnt/e/RMULArtifacts/pb2025_nav_ws
python3 scripts/rmul2027/run_timing_experiment.py --record-dir "$RMUL_TEST_ARTIFACT_ROOT/full-$(date +%Y%m%d-%H%M%S)" --params-file scripts/rmul2027/config/nav2_test.yaml --gz-world-path scripts/rmul2027/config/gz_world_test.yaml --field-contact cad-flat-floor --render-mode gpu --last-case 07-occupied --dds-profile scripts/rmul2027/config/fastdds_local_64m.xml
```

快速对照只把params-file换成nav2_original_speed_early_brake.yaml。完整路线为静止、6运动点、占用目标拒绝，任一失败停止后续目标；180秒／0.35m时序对齐误差、真值位移及实际停车判据保持。真值只用于评估。

运行保护、坐标语义、软件停止和传感器故障注入见[结构性说明](STRUCTURAL_SAFETY.zh-CN.md)。静态绕障及完全封路夹具都需要实际雷达识别，不是把障碍预先写进地图；正式接触计数须为0，接触传感器先做正对照。没有动态横穿测试。

TF2用官方0.25.24项目局部核心库处理已取证的双线程死锁，不替换系统ROS。依赖补丁基准、来源及hash在patches和assets。共享代码保护不能替代硬件急停或MCU看门狗；真实驱动、尺寸、延迟和制动仍需独立验收。

[最新C038报告](reports/C038-RESULTS.zh-CN.md)、[C036未采用候选](reports/C036-RESULTS.zh-CN.md)、[C034历史](reports/C034-RESULTS.zh-CN.md)、[C025定位报告](reports/C025-RESULTS.zh-CN.md)、[C024保护说明](reports/C024-RESULTS.zh-CN.md)、[历史C022报告](reports/C022-RESULTS.zh-CN.md)、[C021大点云传输与静态避障](reports/C021-RESULTS.zh-CN.md)、[C015基础场景结果](TEST_RESULTS.zh-CN.md)分别记录范围。旧说明保留在history。C盘唯一read.md维护AI交接，本仓库文档维护可发布工程快照。


## 存储与收尾（2026-10-21起强制）

本项目曾因整条路线重复录制多路点云耗尽宿主D盘。普通启动、完整路线、故障注入和避障入口均通过storage_entry进入storage_guard；默认不录bag，禁止--record-pipeline整路线原始录包。需要单次原始诊断时使用record_raw_window.py，128MiB或10秒先到即停。

本机工作区的.rmul-storage.json不进Git，指定宿主分区/mnt/d、产物根/mnt/e/RMULArtifacts/pb2025_nav_ws、状态/mnt/e/RMULStorageState/pb2025_nav_ws及Windows Python。队友应根据虚拟磁盘所在分区配置，不能照抄盘符。原生Linux默认检查工作区所在分区；WSL的df /不能代替Windows宿主剩余空间。

开始至少50GiB，40GiB预警，30GiB或容量读取失败停止本轮自有进程；单轮512MiB、累计产物根4GiB，宿主分配增量2GiB。触线返回74，拒绝启动73。阈值用于提前停止，不是硬配额；通知可能被Windows焦点助手隐藏。守护只在受保护入口存活时有效，不是后台永久监控。详见[存储保护说明](STORAGE-SAFETY.zh-CN.md)。

| 文件类别 | 每轮及最终交付后的处理 |
| --- | --- |
| 源码、地图、必要资源、补丁、可复现命令 | 保留，纳入对应版本管理 |
| 结果JSON、关键图、报告与失败索引 | 保留小型必要证据，记录对应配置hash |
| 已通过且摘要完整的重复原始包 | 确认不再需要回放后清理，记录路径、字节和原因 |
| 未查清失败的唯一原始证据 | 保留有诊断价值的最小范围，不自动当作缓存删除 |
| 独立恢复构建目录、临时检出、一次性脚本、重复副本 | 保存验证摘要后删除本轮可重建内容 |

用户已要求每轮及时清理、最终仿真调试完成后清理无用检测数据。执行前核对归属和路径；只处理本项目自建且确认无用的内容，不清理无关项目或用户文件，不通过删除证据来伪造验收通过。删除WSL内部文件与Windows实际可用空间回收分别统计，VHD压缩另需安全停止WSL后执行。

### 固定障碍与单物体对向验收

先确认没有其他仿真。以下工具自动经容量守护、默认不录bag，并在E盘编译小型碰撞诊断代理；需要g++、pkg-config及ignition-transport11/msgs8开发包。固定矩阵四类各3次；移动工具要求同一参数的固定验收通过文件，执行一个预设对向场景3次。以下按仓库根目录运行，并先加载ROS、工作区与本项目DDS配置。

```bash
export RMUL_TEST_ARTIFACT_ROOT=/mnt/e/RMULArtifacts/pb2025_nav_ws
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
source scripts/rmul2027/configure_dds.sh
export RMUL_FIXED_RECORD="$RMUL_TEST_ARTIFACT_ROOT/fixed-$(date +%Y%m%d-%H%M%S)"
python3 scripts/rmul2027/run_fixed_obstacle_acceptance.py --record-dir "$RMUL_FIXED_RECORD"
python3 scripts/rmul2027/run_moving_obstacle_acceptance.py --record-dir "$RMUL_TEST_ARTIFACT_ROOT/headon-$(date +%Y%m%d-%H%M%S)" --fixed-acceptance "$RMUL_FIXED_RECORD/static-acceptance.json"
```

移动代理是0.58×0.50×0.50m、带真实碰撞和激光回波的运动盒体。双方额定平移0.5m/s，物体按仿真时间固定直线行驶；我方目标实际接收后才触发运动。每次先验证运动碰撞正对照，位姿真值只用于驱动和评估，不注入导航。成功到点、仍在自旋、全零安全停和测试保护分别记录。

旧普通配置在`history/before-C031-nav2_test.yaml`，C029中间候选在`history/C029-spin-persistence-nav2_test.yaml`。C031永久2D层在`history/C031-persistent-rplidar-nav2_test.yaml`；三者用于历史对照。当前真实LaserScan经`rplidar_obstacle_cloud.py`保留原时间戳/坐标投影为占用点，再由已有IntensityVoxelLayer每轮重建，观测保留2秒，避免移走后的永久旧格。此链路不使用物体真值。保持50/40/30GiB、单轮512MiB及累计预算；不能以换目录绕过守护。
C038先用LaserScan源时间戳TF到base_footprint，剔除与双图相同未膨胀footprint内/边界的自身回波；外部点继续原frame/时间戳投影，TF缺失不回退未过滤数据并计数。转换器通用默认过滤关闭，普通配置显式启用。参数组rplidar_obstacle_cloud与TF命名空间由入口接入；不把物体真值输入导航。

C038的min_approach_linear_velocity=0.05m/s也参与曲率速度下界，本轮规定避障已重新验证，fast未改。
