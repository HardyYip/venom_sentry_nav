# 本机基线验收记录

日期：2026-10-13（Asia/Shanghai）。环境：D 盘 WSL Ubuntu-22.04、ROS 2 Humble、Gazebo Fortress 6.18.0。所有运行均无 RViz / Gazebo GUI，未连接实车，未使用旧 Venom 工作区。

## 编译与依赖

- 主构建：29 packages finished，10 分 20 秒；无编译失败。完整日志留在 `/home/hardy/pb2025_nav_ws/logs/build-resume.log`。
- 两个仿真兼容修复应用后，增量重编译：2 packages finished，35.8 秒；日志 `/home/hardy/pb2025_nav_ws/logs/build-compat.log`。
- 交付的 `build.sh` 入口实际执行验证：29 packages finished，24.2 秒（已有构建缓存）；日志 `/home/hardy/pb2025_nav_ws/logs/build-entry-verified.log`。
- rosdep check：All system dependencies have been satisfied。
- 外部 Git 版本和导航子模块均固定；原导航 YAML 与算法源码未修改。
- 安装产物的 71 个 ELF 文件动态库检查全部通过，没有 missing library；结果见 `results/ldd-audit.json`。重跑前必须 source Humble 和本工作区，否则会产生环境未加载造成的假缺库。

## 运行结果

| 项目 | 结果 | 原始记录 |
| --- | --- | --- |
| 清场后的建图 | PASS；地图保存成功 | `results/mapping-clean.jsonl` |
| 导航，slam=False | SUCCEEDED；底盘 ground truth 移动 1.013841 米；估计终点误差 0.164200 米 | `results/nav-clean.jsonl` |
| 完全关闭 WSL 后重启导航 | SUCCEEDED；底盘 ground truth 移动 0.721351 米；估计终点误差 0.008920 米 | `results/nav-restart.jsonl` |
| 最终增强检查的建图复测 | PASS；包含原始 Point-LIO 注册点云、适配里程计和注册点云的连续输出检查 | `results/mapping-final.jsonl` |
| 重复启动 | 新实例被锁拒绝，没有启动第二套仿真 | 终端确认 `A headless test is already running` |
| 停机 | 核查启动树退出，无残留 Gazebo、Point-LIO、Nav2 container | 运行进程核查 |

导航短目标距离为 0.8 米。验收同时检查 action 成功、非零最终 cmd_vel、真实仿真底盘 ground truth 位移和估计定位误差，不能用“画出路径”或“收到速度”代替运动成功。位移与目标距离不完全相同，因为一项来自仿真 ground truth，另一项来自定位估计，且机器人有闭环运动及停止过程；上表不是定位精度标定结果。

四个检查的核心生命周期节点为 controller_server、planner_server、bt_navigator 和 velocity_smoother；均返回 active(3)，每个节点仅一个实例。检查 map→base_footprint、odom→base_footprint 可查询，且时钟、LiDAR、IMU、地形点云等有持续数据。导航的 /map 为加载的官方静态地图，不要求持续重复发布。

临时建图输出保存在 `/home/hardy/pb2025_nav_ws/artifacts/mapping_clean.*` 和 `mapping_final.*`，未用于替换官方导航地图。导航加载官方 rmuc_2025.pcd（3787687 点）和同名 PGM/YAML，哈希见部署说明。

## 过程中遇到的失败与处理

1. 最初仿真因为裁判程序缺 shebang 无法执行，以及装甲灯条删除/重建同名 visual 引发渲染崩溃。两处兼容补丁修复后重编译并重测，没有关闭传感器或删掉机器人组件绕过。
2. 旧停机逻辑可能留下 detached 的 Nav2 container；发现后明确作废那一轮受污染的导航结果，增加按启动树清理，并完全关闭 WSL 后重新验收。
3. 组件启动期间曾有 lifecycle 状态查询一直未返回；原检查程序会卡在该次请求。增加查询超时重试，仍要求真实 active 响应。该轮未通过的检查日志保留本机，不算成功。
4. 对 Point-LIO 原始大点云的 BEST_EFFORT 检查订阅没有收到数据，但 loam_interface 和 RELIABLE 临时订阅能收到。最终检查改为与真实消费者一致的 RELIABLE QoS 后通过，生产端 QoS 和导航代码没有改变。

完整过程（包括失败日志）留在 `/home/hardy/pb2025_nav_ws/logs`；仓库只保存上表的成功检查输出和小型依赖审计。公开结果中不包含密码、令牌或大地图数据。

## 这次不能证明什么

没有进行全场长时间巡航、狭窄通道、多机器人、动态障碍压力、定位精度标定或实车安全验收。软件渲染下仿真可能低于实时速度。此结果证明本机上官方导航主链路及重启后短距离导航可运行，不意味着已经完成战队适配。
