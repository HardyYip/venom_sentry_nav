# RMUL 2.00 m/s 贴边、重规划和 IMU 恢复候选

候选取自 2026-11-22 晚间在 Ubuntu 隔离 Gazebo 上实跑的 `c072-verify-imu-stall.QvPlSR`。正式固定障碍和对向测试已经跑完，对向 **2/3**。正式默认仍为 `b5e08a7`。

| 同一构建的场景 | 结果 |
| --- | --- |
| 固定障碍 | 12/12 到点停稳 |
| 对向移动障碍 | 2/3 到点停稳。对向车设定 0.50 m/s |
| 连续长路线 | 7/7 到点停稳，墙箱接触为零 |
| 膨胀层向外脱困 | 3/3 通过。脱困阶段的控制器平移指令峰值均为 0.600 m/s |
| 手动目标坐标对照 | Ubuntu 夹具的两种目标及中止场景再次复跑，合计 3/3 到点停稳 |
| 启动与恢复 | 冷启动 IMU、组件响应超时、连续故障和延后里程计四项夹具退出码均为 0 |

`headon-01` 的导航目标中止。控制日志有 DWB 无有效轨迹和全局重规划失败，最小物理净距为约 0.056 m，接触为零。固定和对向正式 15 场的有效参数、库指纹和进程清理已经核查。原自动脚本的逐项退出表停在最后一场脱困之前，最后一场脱困由该场的原始结果 JSON 确认通过。本候选的移动中主动 IMU 注入回归没有完整回执。原 Windows 手动场景及实车验证仍待完成。[原始退出码](evidence/qualification-exits.txt) · [逐场回执](evidence/ubuntu-validation.json) · [正式审计](evidence/qualification-summary.json)

## 改动

- 贴边选轨加入方向制动可行性评分，制动距离计入位姿采样延迟。向外脱困平移指令上限为 0.60 m/s，继续使用扫掠和净空检查。
- 全局路径短时失效时停止跟随旧路径，等待正常地图更新，每次等 1 秒，最多重试三次。已有规划器夹具覆盖了可达、封死、短时无路恢复和持续无路安全失败。
- 对启动阶段的组件响应超时和 IMU 故障增加受控重启处理。导航放行继续要求独立停车和定位校验。

## Ubuntu 复现

在既有 C072 的 ROS 2 Humble 和 Gazebo 隔离归档中，将本目录保持原名放入证据根。参数里的行为树路径需要容器内挂载为 `/evidence/c072-verify-imu-stall.QvPlSR`。在干净的归档副本执行 `bash build.bash` 使用工作区离线依赖构建，随后用 `bash run-standard-round.bash` 跑正式测试。连续路线用 `bash run.bash` 和 `python3 review.py`。脱困用 `bash run-rviz-inflation-test.bash X Y`，三组起点为 `3.801 -1.942`、`3.785 -1.942` 和 `3.788 -2.100`。

`evidence/build-inputs.json` 保存 Ubuntu 本次原始运行的源码、参数和运行库哈希。速度对照以归档里的前一版贴边候选 `c072-manual-edge-selection.I2C1YT` 为基准。[构建指纹](evidence/build-inputs.json) · [运行库核对](evidence/runtime-libraries.json)
