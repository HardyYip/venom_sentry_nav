# C-021：大点云传输修正与2.5m/s静态导航复核

Codex，2026-10-20T22:17:20+08:00。已接续DeepSeek D-001的离线分析。仅验证北极熊原机器人在2027场地的基础静态导航及临时静态绕障；无动态横穿、无Venom实车适配。

## 找到的问题与本轮改变

补录复现时，原始livox/lidar约19.13Hz且仿真stamp连续0.05s，但转换云录包仅约2.70Hz、LIO约4.09Hz，底盘cmd_vel断续0.832s。loam转发与LIO输出逐帧对齐，原始GPU激光与桥接不是这次断点。

转换点云每帧约1.831MB；本机FastDDS 2.6.12默认SHM段512KiB。同尺寸20Hz、160帧的独立进程对照：默认16/160，显式512KiB 32/160，64MiB两次均160/160。除容量外XML相同，支持该传输容量是本机大点云丢失的主要触发因素。[FastDDS同版本官方说明](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/shared_memory/shared_memory.html)也指出段小于消息会有丢失风险。

新增项目局部config/fastdds_local_64m.xml，仅由可选入口和测试进程加载。保留原速度上限2.5m/s、提前2m减速的B参数，未改控制增益、转弯策略、轮胎、地形、点时间、队列或0.35m阈值；系统ROS与全局DDS设置没变。

另外修正验收工具：发目标前有限等待map→gimbal_yaw_fake完整链的共同TF时间，最多10s，缺链仍失败。原查询已经是Time(0)，问题不是显式使用旧时间，而是新缓冲尚未具备完整链；不是导航代码修改。

## 所有结果，包括未通过的记录

| 记录 | 结果 |
| --- | --- |
| pipeline-static-01 | 不加DDS候选的补录复现，ABORTED，真值终点误差0.565031m。 |
| dds64-static-01 | 通过，终点误差0.112886m，最近足迹间距0.213280m，临时障碍接触0、移走后清除。 |
| dds64-static-02 | 通过，终点误差0.072009m，最近足迹间距0.246574m，临时障碍接触0、移走后清除。 |
| dds64-full-01 | 全新完整8/8：就绪、6运动目标、1占用目标拒绝。 |
| dds64-full-02 | 未完成；前5个运动目标通过，第6个发目标前验收脚本TF查询异常。无该目标UUID，后续未运行，保留原失败。 |
| dds64-full-03 | 仅修验收等待后的全新完整8/8，补足第二次完整通过。 |
| fast-entry-smoke | 首次独立入口检查因过早查插件库中断，实际2.5m/s及2m参数正确；记录保留，不能报通过。 |
| fast-entry-smoke-02 | 改为就绪后查库，独立快捷入口就绪＋近点2/2通过，实际局部DDS与TF2加载已核验。 |

完整通过的两组共12个运动目标，真值最大终点误差0.098590m、估计最大误差0.079349m，最大采样停车速度0.000115m/s，实际指令峰值2.550965m/s。每目标180秒限制与0.35m保护未放宽；完整两组的回退与空局部路径日志均为0。

静态障碍不在原地图中，依靠实际雷达进入代价图；接触传感器先做正对照。接触0只指临时障碍，不是场地全部接触力测量。真值仅用于评估。

## 启动与恢复

从仓库目录，终端A：

```bash
bash scripts/rmul2027/start_rmul2027_fast.sh
```

该入口是2.5m/s＋提前2m减速＋64MiB局部传输，默认cad-flat-floor／auto。实际通过场景为cad-flat-floor、GPU、原圆柱轮；其他CAD碰撞或软件渲染不在本次高速结论内。不会发目标，不打开Gazebo GUI。必须使用rmw_fastrtps_cpp，缺文件会报错。

终端B，需要手测时才开单个RViz：

```bash
FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/scripts/rmul2027/config/fastdds_local_64m.xml" \
  bash scripts/rmul2027/start_rviz.sh
```

停止后回到旧低速入口：

```bash
python3 scripts/rmul2027/stop_rmul2027.py
bash scripts/rmul2027/start_rmul2027_test.sh
```

普通start_rmul2027_test.sh仍保持0.5m/s，未静默替换；严格原速度＋1m减速入口仍是历史失败候选。

自动静态复测在原命令基础上加：

```text
--params-file scripts/rmul2027/config/nav2_original_speed_early_brake.yaml
--dds-profile scripts/rmul2027/config/fastdds_local_64m.xml
--record-pipeline
```

完整路线run_timing_experiment.py同样使用B参数、--dds-profile和--last-case 07-occupied。record-dir须新建，单套后台仿真，先停止手动运行，不与手动目标并发。

## 证据、发布状态与限制

大录包和源码备份：`/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/diagnostics/pipeline-gap-20261020-214927-C021`。汇总C021-summary.json与三张图在本报告旁。

两次静态组消息连续性已改善；第一组LIO与转换云约19Hz，底盘最大间隔0.104s。第二组受RTF影响约15.8墙钟Hz，raw仿真stamp连续；录包LIO接收仍有0.672s空档但header连续、转发同帧有时更早抵达录包，不能把接收时间当成计算时间。第二组cmd_vel仍有0.300s间隔，未宣称所有时延归零。独立64MiB首轮也存在启动接收间隔，后续重复最大0.052s；运动在就绪检查后开始。

指令超时停车保护尚未加入运行代码；合成点时间契约和WSLg永久显示问题未处理。本轮没有重写算法，也没有测试任意地形、所有尺寸障碍、动态横穿或实车。这里的通过是指定静态路线的基础仿真通过。

C-020、C-021新增文件仍只在本地，未提交或推送；GitHub HEAD仍c4381c5。未做本轮干净克隆／第二台电脑验证；旧rmuc_2025未在C021重跑。原低速入口、依赖源码及上游算法未变；不能把C015旧验证当作新交付验证。

补充核验：2.5m/s是控制器参数上限，本轮最终cmd_vel合速度曾短时达到2.550965m/s；没有另外添加合速度硬钳制，不能把配置值描述为严格的物理速度上限。图表已目视检查并扩大坐标范围以完整显示第二次绕行。read.md历史、参数SHA与无残留进程已复核。
