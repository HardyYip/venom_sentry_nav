# RMUL MID360 + Odin1 导航交接

更新日期 2026-11-22（北京时间）。手动测试和下轮需求以当天收到的 ToDesk 交接记录为准。

## 当前基线

- 正式 2.00 m/s 仿真基线为 `b5e08a78e7feb90f54eb95d33d70c99e9a395e29`。正式分支为 `sim/rmul2027-mid-odin-20261121`。速度反馈分支 `sim/rmul2027-speed-feedback-20261122` 也已包含该提交。`6ccd713` 的固定障碍 9/12 属于早期候选，`b4a4657` 是之前的合格基线。[正式验收结果与复现方法](https://github.com/HardyYip/venom_sentry_nav/blob/b5e08a78e7feb90f54eb95d33d70c99e9a395e29/scripts/rmul2027/candidates/mid-odin-60deg/speeds/2.00mps/revisions/c072-speed200-motion-clock-20261122/RESULTS.zh-CN.md)
- 16:37 完成的 Ubuntu 隔离 Gazebo 验收使用同一构建。固定障碍 12/12、对向 3/3、长路线 7/7、膨胀区脱困 3/3。固定、对向和长路线均无墙箱碰撞并通过停稳检查。三组脱困使用 0.20 m/s 指令上限。反向 IMU 中断恢复 3/3，首启 145 ms IMU 缺口触发一次受控重启并通过放行。正式 15 场的有效参数和库哈希已核对。末级指令峰值 2.000 m/s，Gazebo 真值单次采样峰值 2.011 m/s。[逐场验收凭证](https://github.com/HardyYip/venom_sentry_nav/blob/b5e08a78e7feb90f54eb95d33d70c99e9a395e29/scripts/rmul2027/candidates/mid-odin-60deg/speeds/2.00mps/revisions/c072-speed200-motion-clock-20261122/evidence/offline-qualified-20261122.json)

## Windows RViz2 手动测试

- 17:29，`b5e08a7` 从 `(0.060, -0.745)` 向可通行目标 `(9.597, -5.100)` 行驶，在 `(0.966, -4.472)` 触发 `Failed to make progress`，当时距目标约 8.65 m。日志同时记录了 0.15 m 容差下的规划失败、NavFn 回退、反复制动和控制周期低于 20 Hz。各因素的先后与因果关系仍待复核。
- 17:31，下发另一目标 `(9.597, -5.388)` 后成功到点，停在 `(9.540, -5.337)`，随后约 5 秒里程计保持 0 m/s。此次测试也记录到沿膨胀边缘反复减速。
- 复测沿用 Windows + Ubuntu-22.04 WSL 的 `RM-RViz2-LATEST` 完整启动包。仅看图使用 `Open-RViz2.cmd`。手动选点先运行包内 `ensure_wslg_display.py`，再走 `rm-decision-replay/run.bash manual`，以加载 `decision-manual.rviz`、可见 `2D Goal Pose`、日志出现 `MANUAL_READY` 为就绪依据。原始手动记录见回放目录的 `replay.log`、`manual-controller.log`、`manual-planner.log` 和 `manual-bt.log`。

## 待完成任务

1. **复现目标途中中止。** 先在 Ubuntu 用 `b5e08a7` 和相同夹具复跑上述两个目标，同步记录目标状态、全局路径及 NavFn 回退、局部净空、速度指令和制动触发。以异常目标可重复到点并停稳作为修复验收。
2. **定位贴边走走停停。** 对齐原始障碍净空、沿边和朝向障碍的速度分量、控制器输出及末级指令，核实同时置零两个平移轴的条件。修复后确认安全贴边段能连续前进，同时通过碰撞、净空和停车检查。
3. **核对 20 Hz 控制周期与重规划。** 记录控制循环用时、重规划频率和减速时间，确认是否出现规划更新与控制超时重叠，以及对停滞的影响。
4. **将向外脱困平移指令上限从 0.20 提至 0.60 m/s。** 仅用于已进入膨胀区、停车并通过车身扫掠和净空检查后，沿已验证路径向外驶出的阶段。复测安全退出、占据或深层危险目标拒绝、停稳及后续接单。普通贴边掉速按第 2 项复核。0.60 m/s 待实施和验证。
5. **完成修复后的同版回归。** 先过定向用例，再跑固定 12、对向 3、脱困 3、长路线 7，并复核 IMU 恢复、接触、速度及一秒停稳。最后用原 Windows 启动包手动验证两个目标和贴边连续通行。
6. **完成实车验收。** 实车的传感器时序、定位、净空、速度跟踪和故障停车仍待现场验证。

本次仅整理文档和核对已有证据。Windows 原始日志未在本次 Ubuntu 会话读回，复核沿用 17:29–17:31 的现场记录。
