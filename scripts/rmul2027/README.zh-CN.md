# 北极熊 × RMUL 2027 3V3 基础导航仿真

目标是验证北极熊原机器人在2027场地中的基础定位、规划、控制和静态避障。没有Venom实车适配、动态横穿或比赛战术开发。场地来源为官方RMUL2027_3V3_V1.0.0.stp。

C022增加运行时输入／位姿／同步时效保护、故障停车覆盖自旋、恢复防旧指令重放、诊断和可复现故障注入。两次全新快速配置完整路线、普通0.5m/s路线、静态绕障、完全封路停车与移除后新目标恢复、2025旧场景回归、干净检出构建与默认冒烟均已通过，数据见[最新报告](reports/C022-RESULTS.zh-CN.md)。点时间快照候选未通过，默认未采用；不要把基础仿真通过理解为所有结构问题均已解决。

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

关闭RViz后可以重开，单实例锁防止重复启动。WSLg下窗口可能以最小化或越界位置出现，`start_rviz.sh`会自动后台调用`restore_rviz_window.ps1`，30秒内轮询并恢复本项目窗口；该调用不能加`-WindowStyle Hidden`、也不能把stdio指向已关闭的fd，否则会返回`Invalid argument`或超时。设`RMUL2027_RESTORE_WINDOW=0`可关闭自动恢复。若30秒内未恢复，手动执行：

```powershell
powershell -ExecutionPolicy Bypass -File restore_rviz_window.ps1 -ConfigTitle "\\wsl.localhost\Ubuntu-22.04\home\hardy\pb2025_nav_ws\src\pb2025_sentry_nav\scripts\rmul2027\config\interactive-light.rviz"
```

这只是显示层恢复，与导航链路无关；WSLg无窗口／COPY MODE仍属独立显示传输问题，不能靠重启导航处理，也不承诺跨Windows／WSL重启永久无故障。

## 场景和未通过项

| 场景 | 含义／边界 |
| --- | --- |
| cad-flat-floor（默认） | 中央薄标识移除碰撞＋两处3mm地板碰撞接平；属于明确列出的仿真接触近似 |
| cad-flat-marking | 只移除中央薄标识碰撞；历史精确拐角仍有地板拼接阻塞 |
| cad | 保留全部原CAD碰撞，保留历史失败复现 |
| seam-ramps | 历史12段斜面过渡近似 |

原CAD、视觉、地图及历史失败均保留。不宣称官方确认薄层真实铺设方式；也没有开放高台、隧道或任意三维通行。

Point-LIO默认仍沿用legacy_rolling＋timestamp_unit=2的已知失配。新增显式point_time_is_valid与instantaneous只供独立契约／失败复现；nav2_point_time_snapshot.yaml远点误差0.368m未通过，未切默认。实车需核对真实雷达每点时间格式和外参，不采用模拟器快照假设。

## 自动验收与证据

先停止已有仿真，source Humble与../../install/setup.bash。record-dir须不存在，每次只运行一组；大型录包放工作区artifacts。普通完整路线：

```bash
python3 scripts/rmul2027/run_timing_experiment.py --record-dir /tmp/rmul2027-full-new --params-file scripts/rmul2027/config/nav2_test.yaml --gz-world-path scripts/rmul2027/config/gz_world_test.yaml --field-contact cad-flat-floor --render-mode gpu --last-case 07-occupied --dds-profile scripts/rmul2027/config/fastdds_local_64m.xml
```

快速对照只把params-file换成nav2_original_speed_early_brake.yaml。完整路线为静止、6运动点、占用目标拒绝，任一失败停止后续目标；180秒／0.35m时序对齐误差、真值位移及实际停车判据保持。真值只用于评估。

运行保护、坐标语义、软件停止和传感器故障注入见[结构性说明](STRUCTURAL_SAFETY.zh-CN.md)。静态绕障及完全封路夹具都需要实际雷达识别，不是把障碍预先写进地图；正式接触计数须为0，接触传感器先做正对照。没有动态横穿测试。

TF2用官方0.25.24项目局部核心库处理已取证的双线程死锁，不替换系统ROS。依赖补丁基准、来源及hash在patches和assets。共享代码保护不能替代硬件急停或MCU看门狗；真实驱动、尺寸、延迟和制动仍需独立验收。

[最新C022报告](reports/C022-RESULTS.zh-CN.md)、[C021大点云传输与静态避障](reports/C021-RESULTS.zh-CN.md)、[C015基础场景结果](TEST_RESULTS.zh-CN.md)分别记录范围。旧说明保留在history。C盘唯一read.md维护AI交接，本仓库文档维护可发布工程快照。
