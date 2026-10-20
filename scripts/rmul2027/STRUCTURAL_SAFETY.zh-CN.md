# C022：运行保护、坐标契约与故障验收

这些修改针对结构性逻辑。0.5秒等门限只在当前仿真验证，需要按真实输入延迟、制动预算标定；没有Venom实车适配。

## 运行节点改变

fake_vel_transform默认启用safety_enabled。监视命令接收、里程计接收、源时间戳进展／年龄和局部路径同步；识别空路径、无效输入和未来时间。故障时20ms墙钟定时器持续输出vx=vy=w=0，覆盖自旋；诊断safety_status报告STOPPED及具体原因。恢复后等待新鲜输入和新的速度指令，不重放旧缓存。普通导航零速在健康additive模式仍允许请求的自旋，与故障停车区分。

原零速分支没有替换缓存，后续同步可能重放非零速度；已修复。虚拟TF现在使用里程计的原时间戳，拒绝过期／冻结／超前姿态，不能用当前时间掩盖旧姿态。

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| safety_enabled | true | 运行保护；false只供历史对照 |
| command_timeout | 0.5s | 速度输入接收超时 |
| pose_timeout | 0.5s | 位姿接收、源年龄及stamp停更门限 |
| sync_timeout | 0.5s | 控制器活跃时路径／同步超时 |
| future_stamp_tolerance | 0.1s | 源时间超前容差 |
| max_sync_stamp_skew | 0.2s | 配对里程计与路径时间差 |
| diagnostics_period | 1s | 无状态变化时诊断周期 |
| rotation_mode | additive | 保留角速度＋请求自旋；heading明确禁止叠加自旋 |

软件停止话题为example_interfaces/msg/Bool，默认safety_stop。true立即停车；false解除后仍须新命令。它不是硬件急停，不能在保护节点本身崩溃、底盘绕过该节点接收其他发布者或MCU失效时保证停车。实车必须另查驱动看门狗、仲裁及硬件急停，当前未知。

## 坐标与旋转

保留原二维旋转：机体vx=vx输入*cos(yaw)+vy输入*sin(yaw)，机体vy=-vx输入*sin(yaw)+vy输入*cos(yaw)。独立ROS测试覆盖横移、同时vx/vy和±90°／180°等朝向。heading模式验证外部朝向闭环不会被独立spin常量偏置；测试闭环是受控运动学夹具，不是证明Nav2在当前enable_rotation=false配置中已完成朝向闭环，也不是实车调参。

## 可复现测试

先关闭本工程已有仿真，source Humble及本工作区install/setup.bash，在仓库目录执行。每个record-dir须为新目录，以下命令会主动驱动机器人或注入故障。

```bash
ROS_DOMAIN_ID=178 python3 scripts/rmul2027/test_velocity_safety.py --rotation-mode additive --record-dir /tmp/guard-additive-new
ROS_DOMAIN_ID=179 python3 scripts/rmul2027/test_velocity_safety.py --rotation-mode heading --record-dir /tmp/guard-heading-new
bash scripts/rmul2027/run_point_time_contract.sh
python3 scripts/rmul2027/run_velocity_fault_sim.py --record-dir /tmp/physical-fault-new
bash scripts/rmul2027/build_cart_test_tools.sh
python3 scripts/rmul2027/run_cart_obstacle_test.py --record-dir /tmp/parked-new --scenario parked --params-file scripts/rmul2027/config/nav2_original_speed_early_brake.yaml --actor "$PWD/../../artifacts/rmul_2027_3v3/tools/obstacle_cart_actor" --dds-profile scripts/rmul2027/config/fastdds_local_64m.xml --record-pipeline
```

完全封路沿用上一命令，换新record-dir、--scenario blocked，并加 --obstacle-model scripts/rmul2027/assets/test_barrier.sdf --obstacle-size 0.14 14 0.5。障碍为静态几何夹具，不是新增对手机器人、动态横穿或战术。它必须先进入雷达点云和代价地图；接触传感器先做正对照。自然ABORTED、实际停车且无正式接触后移除，双代价地图清除，再显式下发新目标验证恢复。测试取消不算自然失败通过。

## 点时间：实验接口已补，导航问题仍保留

仿真转换器旧输出time为秒，当前稳定配置按US读取，存在已知失配。Gazebo Sensors6的GPU雷达一次Render后以同一_now标记整帧，原字段无每点时间，因此为该模拟器提供instantaneous快照模式是基于实现的推断，不能照搬真实雷达。依据：[Gazebo原始实现](https://raw.githubusercontent.com/gazebosim/gz-sensors/ign-sensors6/src/GpuLidarSensor.cc)。

新增转换器point_time_mode，默认legacy_rolling不变；instantaneous输出零偏移。Point-LIO新增preprocess.point_time_is_valid=false；显式true才把全零时间视为合法provided-time，不再按方位角合成扫描时间，缺失字段及非法偏移明确拒绝。仅适用于VELO16输入分支，其他雷达保持原实现。

11项实际转换器／预处理测试通过。但独立nav2_point_time_snapshot.yaml远点触发0.368m误差保护，尚未通过导航，未采用。不能只改timestamp_unit宣称修好；进一步需要分析Point-LIO快照批量更新与时钟，当前不重写滤波器。候选仅用于失败复现，默认与fast入口继续使用已验证legacy模式。

## 传输及边界

普通、fast与RViz入口共用configure_dds.sh，为本进程及子进程加载64MiB FastDDS共享内存配置，并记录hash；不改系统ROS或全局DDS。普通入口保持0.5m/s，fast仍是2.5m/s＋提前2m减速。RMUL2027_DDS_PROFILE可显式选择其他配置，但须另验收。64MiB是本机1.831MB点云的已验证设置，不是适用于所有机器的固定标定值。

完整结果以reports/C022-RESULTS.zh-CN.md为准。WSLg显示独立于导航链路，历史存在共享内存EIO／COPY MODE；禁止用反复启动导航修复画面。`start_rviz.sh`现会在WSLg下自动后台调用restore_rviz_window.ps1，30秒内轮询并恢复最小化／越界窗口，实测4秒内生效；该自动调用不得加`-WindowStyle Hidden`或把stdio指向已关闭fd，否则返回`Invalid argument`／超时。自动恢复只处理显示层，超时可手动调用同一脚本，WSLg传输本身跨系统重启的永久修复仍未证明；导航保护与窗口恢复是两件事。
