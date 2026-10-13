# Venom 导航：北极熊原样复现基线

这是对 SMBU PolarBear `pb2025_sentry_nav` 的个人部署分支，不是从零原创的导航算法。原作者、许可证、提交历史、package 名称和子模块引用全部保留。旧 Venom 工作区未修改；本阶段没有接实车、没有加入自瞄算法，也没有针对战队底盘调参。

## 已部署的位置

- Ubuntu-22.04 / ROS 2 Humble，WSL2。
- WSL 虚拟磁盘：Windows `D:\WSL\Ubuntu2204\ext4.vhdx`。
- 工作区：`/home/hardy/pb2025_nav_ws`。
- 导航源码：`/home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav`。
- 导航上游基准：`12c4da5318e114044c9d897dd7044b33470f14ff`。
- 独立仿真仓库：`/home/hardy/pb2025_nav_ws/src/rmu_gazebo_simulator`。
- 外部源码固定于 `dependencies.lock.repos`；导航子模块固定于 `submodules.lock.txt` 和 Git gitlink。
- 本地日志在 `/home/hardy/pb2025_nav_ws/logs`，测试地图在 `/home/hardy/pb2025_nav_ws/artifacts`。

以上 Linux 路径都在 D 盘虚拟磁盘里。WSL 应用本身属于 Windows 系统组件；不要把“Ubuntu 的文件系统在 D 盘”理解成 Windows 不再使用任何 C 盘文件。

## 下次怎么启动

在 **Ubuntu / WSL 终端**执行：

```bash
bash ~/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/venom/start_headless.sh nav
```

若当前是 **Windows PowerShell**，执行：

```powershell
wsl.exe -d Ubuntu-22.04 -- bash -lc 'bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/venom/start_headless.sh nav'
```

不要在 Ubuntu 终端执行 Windows 的 `wsl` 管理命令，也不需要安装 Ubuntu 同名的 `wsl` 软件包。

启动的是官方 `rmuc_2025` 场地，机器人命名空间 `/red_standard_robot1`。Gazebo Fortress 只启动服务器并自动运行，RViz 和 Gazebo GUI 都不会打开。没有窗口是预期行为。脚本自动加载 Humble 和本工作区，不需要手动 source 旧 Venom。

`nav` 使用 `slam:=False` 和官方先验 PCD 做重定位；`mapping` 使用官方 `slam:=True`。一次只能运行一个实例，重复启动会明确退出。按 **Ctrl+C** 停止整套启动树，通常需要等待约 10–20 秒。

建图入口：

```bash
bash ~/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/venom/start_headless.sh mapping
```

## 怎么验证它真的在运行

保持启动终端运行，另开一个 Ubuntu 终端：

```bash
source /opt/ros/humble/setup.bash
source ~/pb2025_nav_ws/install/setup.bash
python3 ~/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/venom/verify_headless.py --mode nav
```

注意：**导航验证会自动发送短距离目标，让仿真机器人移动**，不是只读检查；仅用于本仿真。建图时改为 `--mode mapping`，该模式不发送导航目标。

检查器验证时钟推进、LiDAR/IMU、Point-LIO 的里程计和注册点云、loam_interface、地形点云、地图与 TF；核心导航生命周期必须真实返回 active，并且每个核心节点只允许一个实例。连续采样后再发送安全间距内的短目标。导航验收必须同时满足 action SUCCEEDED、非零底盘速度、仿真底盘 ground truth 实际位移和终点误差要求。

地图是静态或低频话题，不以频率高低判断好坏。原始 Point-LIO 注册点云的检查订阅使用 RELIABLE，与 loam_interface 的订阅一致，避免大消息在 BEST_EFFORT 检查订阅中丢失。生命周期查询有超时重试，不会把“没收到响应”当成已激活。

## 先验地图与源码

导航使用下载的官方 `rmuc_2025.pcd`，不是旧 Venom 点云，也不是本次 smoke test 临时建的图：

- PCD：`/home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/rmuc_2025.pcd`。
- 大小 121206237 字节，3787687 点。
- SHA256：`5e1bda92eacc6bc338b278a7032dfa49cc1a8eb1d1fee83d09d1d726750d717d`。
- 官方配套 PGM/YAML：同一导航仓库的 `pb2025_nav_bringup/map/simulation/rmuc_2025.*`。
- PGM SHA256：`8574dc0f1309dcc4eda06ad22d7ae3b44e6af2f346768b28fb8642cc5f7f0be2`。
- YAML SHA256：`f2dcccc21ed21f21efad18f76240e1adb058d31e86380add37ebc786e21d5e05`。

三者与仿真启动参数都选择 `rmuc_2025`；测试验证了出生区域重定位及短距离导航，但不是对整张比赛地图逐点验收。PCD 不提交 Git；新克隆必须另行下载同一官方资源并核对哈希。

## 这次改了什么

导航源码、Nav2 YAML、规划器、控制器、定位算法、Topic/Action/TF 和速度处理链路没有替换。当前仍为 ThetaStarPlanner、OmniPidPursuitController、small_gicp_relocalization。

新增的只是本目录的无界面启动、检查、停机、依赖清单和说明，以及两份外部仿真仓库的必要兼容补丁：

1. `patches/rmu_gazebo_simulator.patch`：给裁判系统 Python 程序补 `#!/usr/bin/env python3`，修复 launch 启动时的 `Exec format error`。
2. `patches/rmoss_gazebo.patch`：装甲灯条更新颜色时使用 Fortress 的 VisualCmd 原位更新材质，不再删除后立即重建同名 visual，修复无界面传感器渲染线程遇到的同名 visual 竞争及崩溃。保留模型、相机、灯条和裁判功能。

这两份补丁保存在自己的导航 Fork 中，由 `build.sh` 幂等应用到外部依赖；没有向北极熊官方仓库推送。

停机脚本只跟踪本次两个 launch 的子进程，并核对 UID、PID 启动时间，避免结束无关进程。SIGINT 正常退出失败后才逐级终止残留进程。日志里因此可能出现停机阶段的 signal 信息，不等于运行测试失败。

## 在相同环境重新构建

本机已经安装完毕，无需为了启动而重新安装。若以后要在相同的 D 盘 WSL 环境重建，先递归克隆本 Fork 的 `venom/integration` 分支到上述导航源码路径，再运行：

```bash
source /opt/ros/humble/setup.bash
cd ~/pb2025_nav_ws
python3 src/pb2025_sentry_nav/scripts/venom/bootstrap_sources.py
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
python3 -m pip install --user xmacro==1.2.1
bash src/pb2025_sentry_nav/scripts/venom/build.sh
```

Fortress / ros_gz、编译工具、vcstool、rosdep、libomp-dev、libaprutil1-dev 等已安装，具体版本见 `environment.lock.tsv`。`bootstrap_sources.py` 固定八个配套仓库；遇到来源不符或需要切版本但有本地修改，会停止而不是覆盖。已经位于固定提交的源码不会被重置，已有兼容补丁也会保留。

编译使用两路 colcon worker，每路 CMake 单线程；small_gicp 固定源码通过 FetchContent 本地路径传入，避免构建时临时拉取变化的版本。系统 apt 版本清单只是复现记录，不保证软件镜像永久保留对应二进制包。

Git 的 `origin` 指向个人 Fork，`upstream` 指向官方且本地禁止 push。不要提交 build/install/log、PCD、凭据；Point-LIO 退出时可能在自己的子模块目录生成 `PCD/scans.pcd`，这是运行产物，不是源码修改，也不上传。

## 边界

测试结果见 `TEST_RESULTS.zh-CN.md`。这是“官方导航能在本机无界面仿真里正常运行”的基线，不代表比赛级可靠性、狭窄通道能力、实车接口或底盘参数已经验收。WSL 软件渲染仍可能低于实时速度；不启动 GUI 不等于激光传感器完全不需要渲染。

结束学习后，先 Ctrl+C 等待仿真停止，再在 **Windows PowerShell** 中执行 `wsl.exe --shutdown`。这会关闭全部 WSL 发行版，执行前确认没有其他需要保留的 WSL 工作。
