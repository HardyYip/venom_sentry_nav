# RMUL MID360 + Odin1 导航交接

更新日期 2026-11-23（北京时间）。历史手动测试保留当天收到的 ToDesk 交接记录。

## 2026-11-23 离线验收与默认版

- 正式分支为 `sim/rmul2027-mid-odin-20261121`，默认使用本轮 2.00 m/s 的源码和参数。固定 12/12、对向 3/3、长路线 7/7、膨胀区脱困 3/3、移动 IMU 恢复 3/3、启动恢复 4/4 和 WSL 无界面手动坐标 3/3。对向车为 0.50 m/s，正式对向最小物理净距 0.099 m。
- 短时全局无路会停止跟随旧路线并等待有效重规划。低速轨迹增加更细的横向方向，避免原固定定向夹具在膨胀边停滞。向外退出阶段的控制器指令上限为 0.60 m/s。使用最新目标后依旧检查扫掠净空，危险目标拒绝后可继续下发合法点位。
- RViz2 每次沿用 `RM-RViz2-LATEST` 完整启动包和项目内手动启动入口。窗口手动复测和实车验收继续等待现场测试。[正式验收结果与凭证](candidates/mid-odin-60deg/speeds/2.00mps/revisions/c072-verify-headon-planar-accel-20261123/RESULTS.zh-CN.md)

## 11 月 22 日基线

- 正式 2.00 m/s 仿真基线为 `b5e08a78e7feb90f54eb95d33d70c99e9a395e29`。正式分支为 `sim/rmul2027-mid-odin-20261121`。速度反馈分支 `sim/rmul2027-speed-feedback-20261122` 也已包含该提交。`6ccd713` 的固定障碍 9/12 属于早期候选，`b4a4657` 是之前的合格基线。[正式验收结果与复现方法](https://github.com/HardyYip/venom_sentry_nav/blob/b5e08a78e7feb90f54eb95d33d70c99e9a395e29/scripts/rmul2027/candidates/mid-odin-60deg/speeds/2.00mps/revisions/c072-speed200-motion-clock-20261122/RESULTS.zh-CN.md)
- 16:37 完成的 Ubuntu 隔离 Gazebo 验收使用同一构建。固定障碍 12/12、对向 3/3、长路线 7/7、膨胀区脱困 3/3。固定、对向和长路线均无墙箱碰撞并通过停稳检查。三组脱困使用 0.20 m/s 指令上限。反向 IMU 中断恢复 3/3，首启 145 ms IMU 缺口触发一次受控重启并通过放行。正式 15 场的有效参数和库哈希已核对。末级指令峰值 2.000 m/s，Gazebo 真值单次采样峰值 2.011 m/s。[逐场验收凭证](https://github.com/HardyYip/venom_sentry_nav/blob/b5e08a78e7feb90f54eb95d33d70c99e9a395e29/scripts/rmul2027/candidates/mid-odin-60deg/speeds/2.00mps/revisions/c072-speed200-motion-clock-20261122/evidence/offline-qualified-20261122.json)

## Windows RViz2 手动测试

- 17:29，`b5e08a7` 从 `(0.060, -0.745)` 向可通行目标 `(9.597, -5.100)` 行驶，在 `(0.966, -4.472)` 触发 `Failed to make progress`，当时距目标约 8.65 m。日志同时记录了 0.15 m 容差下的规划失败、NavFn 回退、反复制动和控制周期低于 20 Hz。各因素的先后与因果关系仍待复核。
- 17:31，下发另一目标 `(9.597, -5.388)` 后成功到点，停在 `(9.540, -5.337)`，随后约 5 秒里程计保持 0 m/s。此次测试也记录到沿膨胀边缘反复减速。
- 复测沿用 Windows + Ubuntu-22.04 WSL 的 `RM-RViz2-LATEST` 完整启动包。仅看图使用 `Open-RViz2.cmd`。手动选点先运行包内 `ensure_wslg_display.py`，再走 `rm-decision-replay/run.bash manual`，以加载 `decision-manual.rviz`、可见 `2D Goal Pose`、日志出现 `MANUAL_READY` 为就绪依据。原始手动记录见回放目录的 `replay.log`、`manual-controller.log`、`manual-planner.log` 和 `manual-bt.log`。

## 11 月 22 日待办记录

1. **复现目标途中中止。** 先在 Ubuntu 用 `b5e08a7` 和相同夹具复跑上述两个目标，同步记录目标状态、全局路径及 NavFn 回退、局部净空、速度指令和制动触发。以异常目标可重复到点并停稳作为修复验收。
2. **定位贴边走走停停。** 对齐原始障碍净空、沿边和朝向障碍的速度分量、控制器输出及末级指令，核实同时置零两个平移轴的条件。修复后确认安全贴边段能连续前进，同时通过碰撞、净空和停车检查。
3. **核对 20 Hz 控制周期与重规划。** 记录控制循环用时、重规划频率和减速时间，确认是否出现规划更新与控制超时重叠，以及对停滞的影响。
4. **将向外脱困平移指令上限从 0.20 提至 0.60 m/s。** 仅用于已进入膨胀区、停车并通过车身扫掠和净空检查后，沿已验证路径向外驶出的阶段。复测安全退出、占据或深层危险目标拒绝、停稳及后续接单。普通贴边掉速按第 2 项复核。0.60 m/s 待实施和验证。
5. **完成修复后的同版回归。** 先过定向用例，再跑固定 12、对向 3、脱困 3、长路线 7，并复核 IMU 恢复、接触、速度及一秒停稳。最后用原 Windows 启动包手动验证两个目标和贴边连续通行。
6. **完成实车验收。** 实车的传感器时序、定位、净空、速度跟踪和故障停车仍待现场验证。

本次仅整理文档和核对已有证据。Windows 原始日志未在本次 Ubuntu 会话读回，复核沿用 17:29–17:31 的现场记录。

## 22 日晚 Ubuntu 候选交接

`c072-verify-imu-stall.QvPlSR` 已保存该构建的源码、参数与最后一轮原始回执。固定 12/12、长路线 7/7、脱困 3/3，脱困指令上限 0.60 m/s。对向 `headon-01` 中止，正式整轮保留为 2/3 对向的候选记录。后续复核对向失败、移动中 IMU 注入和 Windows 原始手动场景。[候选结果与复现方法](candidates/mid-odin-60deg/speeds/2.00mps/revisions/c072-verify-imu-stall.QvPlSR/RESULTS.zh-CN.md)
