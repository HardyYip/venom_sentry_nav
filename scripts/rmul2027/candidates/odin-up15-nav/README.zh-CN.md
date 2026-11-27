# ODIN 上仰 15° 的独立导航候选

此入口使用本机 Gazebo Fortress 6.18.0、ROS 2 Humble 和无发射组件的全向测试底盘。默认 2.00 m/s 版本另有自己的入口。

从本目录运行以下命令。每次运行创建新结果目录，每场在独立的网络与进程命名空间内冷启动。整轮通过后，审计程序复查逐场轨迹、停车、控制参数、角度和输入指纹。

```bash
./run_nav.bash
```

## 仿真配置

| 项目 | 候选配置 |
|---|---|
| 导航基线 | 11 月 23 日提交 `3eb64d3`，2.00 m/s 平移上限，1.75 m/s² 平面加速度上限 |
| 场地 | 原版 3v3 平地场地网格、静态占用地图、正式用例的箱体尺寸和起终点 |
| 雷达 | 原版 ODIN 与 MID360 的 Gazebo GPU 激光传感器。ODIN 参考轴相对水平上仰 15° |
| 规划与控制 | 原版 ReserveNavfn、局部控制插件、速度平滑器、导航行为树。PoseFresh 检查插件由留存源码重新构建 |
| 点云与定位 | 两路原生仿真点云经高度过滤后进入原版障碍层。底盘姿态来自 Gazebo 里程计，使用已知地图起点 |
| 底盘与传感器支架 | 测试底盘水平截面 0.58 × 0.53 m。支架在起步时将 ODIN 的水平方向对准本段目标，整段保持该方向 |
| 覆盖范围 | 四组固定障碍各三次，另有三次 0.50 m/s 对向移动障碍。每次都重新启动 Gazebo 与 Nav2 |

原始独立传感器文件在 `odin-up15.sdf`。运行配置、输入指纹与阳性碰撞检验在 `native-nav`。依赖缓存在本目录的 `runtime`，同时读取相邻的 11 月 23 日基线目录。本机运行需要可用的 NVIDIA 驱动和用户命名空间。实体管道摆动、原车完整定位、出生点到测试起点的预定位，以及七段连续长路线均属于该组用例以外的验证。

## 11 月 27 日整轮结果

原生 Gazebo 同版固定障碍 12/12、对向 3/3。接触阳性对照通过，正式轮接触数为零。固定箱体最小净距 0.2148 m，对向最小净距 0.0545 m，最高实测速率 1.700 m/s，最大到点误差 0.0775 m，最短连续停稳 1.4 s。详见 [独立审计](native-nav/nav-round-01/audit.json) 和 [逐场汇总](native-nav/nav-round-01/summary.json)。

## 远点与航点决策

固定种子 2294429396 抽取的 5 个远点均到达并停稳，最大终点误差 7.49 cm。临时封路 3.50 s 的单场中，旧路径被取消，车在封路期间连续停稳 2.60 s，随后恢复并到点。普通航点测试通过专用适配器按 Nav2 的实际到点回执连续派发 3 个目标，三次均到达，最大误差 7.77 cm。三组测试都记录了零接触。完整决策节点、实车和摆动遮挡仍待单独验证。详见 [远点与决策审计](native-nav/far-goals-20261127/audit.json) 和 [实际轨迹图](native-nav/far-goals-20261127/routes.png)。

保留了[早期对向探测失败记录](native-nav/field-probe/headon-01/result.json)。该次净距为 4.67 cm，低于 5 cm 门槛。正式轮的 3 次对向用例单独计数。

## 复跑依赖

在已验收的 Linux x86-64 环境中使用本候选。宿主需要 Python 3、NumPy、PyYAML、bubblewrap、可用的用户命名空间和 NVIDIA GPU。进入本目录，按下表放置已有的本机依赖。仓库保存了源码、逐场证据和[依赖指纹](native-nav/preflight.json)。单独克隆仓库还需要恢复运行缓存和基线归档。

| 相对本目录的路径 | 内容 |
|---|---|
| `../c072-verify-headon-planar-accel-20261123/` | 完整 11 月 23 日本机测试归档，包括用例、stack-evidence、场地和 planner-install。可用指向该归档的符号链接。 |
| `runtime/` | 已验收的 Jammy/Humble/Fortress 运行缓存，内含 root、omni、pb_nav2_plugins、field-models、gfx。可用符号链接。 |
| `native-nav/safety-install/lib/` | 与 Humble ABI 匹配的 PoseFresh 插件。可恢复已验收的库，或在同一 Humble 环境中用 CMake 构建并安装本目录的 native-nav/safety-src。 |

复查留存的 15 场记录和轨迹可运行 `(cd native-nav && python3 audit.py nav-round-01)`。复跑五个随机远点可运行 `python3 native-nav/far-goals-20261127/run.py`，结果写入带时间戳的新目录。普通航点测试的适配器源码为 `mission-probe.cpp`，用 C++17 编译并将仓库的 `rmul2027_decision/include` 加入头文件路径，将生成的 `mission-probe` 放在源码旁。已验收的三点记录在 [mission-test](native-nav/far-goals-20261127/mission-test/far-01/result.json)。
