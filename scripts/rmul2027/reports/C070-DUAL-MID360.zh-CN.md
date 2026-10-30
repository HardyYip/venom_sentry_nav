# C070 双MID360最小仿真配置

状态：可选替换及规定最终有限仿真验收通过；首轮恢复后路径失败仍单列未关闭。最终成绩以同目录 C070-RESULTS.zh-CN.md 和固定证据根为准。

这是把原RPLidar A2替换为副MID360的可选仿真入口。保留主MID挂载、单IMU Point-LIO/GICP、C052低速控制及C062/C063/C064/C067安全与时序修复；没有改变悬挂、云台等机构。副MID使用原二维雷达仿真位姿，不能视为最终硬件外参标定。

从WSL启动（默认不打开RViz、不发送目标）：

~~~bash
export RMUL_TEST_ARTIFACT_ROOT=/mnt/c/Users/Hardy/Documents/Codex/2026-10-18/headoff-md1/outputs/dual-mid-C070-artifacts
bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/start_dual_mid360.sh
~~~

导航保留0.3秒 decision_stop 安全心跳：只启动上述导航、尚未运行真实决策/有效比赛输入时会保持停车。决策启动与TEST输入按 rmul2027_decision/README.zh-CN.md 配套操作。固定/移动障碍夹具的 --test-decision-permit 只用于直接Action验收，拒绝已有决策发布者，随夹具故障停止；不能当比赛决策或绕过安全的常驻许可。\n\n仅在显式需要已验受控恢复时设置 RMUL_IMU_RECOVERY=1。它不保证任意位置恢复，后端真实IMU缺口仍锁止，恢复须静止重新初始化及真实GICP质量门禁。

副MID话题为 auxiliary/livox/lidar，帧为 aux_mid360，经源时间TF、地面/自身/范围过滤后输出 auxiliary/obstacles。输出XYZ保持原帧，intensity=1表示占用。副IMU不接入LIO，两张地图继续用原IntensityVoxelLayer/2秒保持。副雷达掉线独立停车机制未新增、未验。

默认创建本轮私有ament资源overlay，生成SDF/桥接/参数和SHA256清单，原description/simulator安装不覆盖。原 start_low_speed.sh 在不设 RMUL_SENSOR_PROFILE 时仍为旧MID＋RPLidar；可选环境 RMUL_SENSOR_PROFILE=dual-mid360 选择新模型。试验入口 test_dual_mid360.sh 明确分开C067决策目录和C062导航基库。

验收入口：

~~~bash
bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/test_dual_mid360.sh low --record-dir "$RMUL_TEST_ARTIFACT_ROOT/your-unique-low-run"
bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/test_dual_mid360.sh long --record-dir "$RMUL_TEST_ARTIFACT_ROOT/your-unique-crossfield-run"
bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/scripts/rmul2027/test_dual_mid360.sh long --five-minute-round --record-dir "$RMUL_TEST_ARTIFACT_ROOT/your-unique-round"
~~~

另有 fixed / moving / imu / timing / persistent 模式。fixed的 --low-box-clearance-check 是独立25cm地面障碍测试（1例），不能替代默认四类×3的12例。moving必须指定同版12例的 --fixed-acceptance。先阅读脚本参数及结果报告，不把示例执行当作已有成绩。

沿用0.5m/s、0.35m/180秒到点和一秒内所有速度/命令分量≤0.03的门槛。一次只运行一套仿真，保留原单实例锁；50/40/30GiB容量门禁、4GiB任务、512MiB/1800秒单轮和2GiB宿主增长保护保持。默认不录bag，源时间消息摘要有界，原始失败不删。

回退：正常结束本轮自有启动树，重新核单实例，再用原低速入口且不设置 RMUL_SENSOR_PROFILE。不要删除锁、reset/clean、覆盖旧安装，或停未知进程。已有可恢复备份和完整指纹见 outputs/agent.md 第8节。

仿真模型是规则32层GPU扫描，不代表真实MID的非重复逐点时序。真车、Orin原生性能、正式裁判协议、fast、视觉射击、悬挂动力学及新定位移植都不属于本次验收。
