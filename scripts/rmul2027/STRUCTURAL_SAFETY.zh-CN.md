# C022：运行保护、坐标契约与故障验收

这些修改针对结构性逻辑。0.5秒等门限只在当前仿真验证，需要按真实输入延迟、制动预算标定；没有Venom实车适配。

## 运行节点改变

fake_vel_transform默认启用safety_enabled。监视命令接收、里程计接收、源时间戳进展／年龄和局部路径同步；识别空路径、无效输入和未来时间。故障时20ms墙钟定时器持续输出vx=vy=w=0，覆盖自旋；诊断safety_status报告STOPPED及具体原因。恢复后等待新鲜输入和新的速度指令，不重放旧缓存。普通C038配置显式spin_on_zero_command=false：健康零Nav2输入也停止追加自旋。此参数的通用默认true保留历史additive自旋行为，heading仍不追加自旋；故障停车不受该选项影响，始终全零。

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

先关闭本工程已有仿真，source Humble及本工作区install/setup.bash，在仓库目录执行。每个record-dir须为新目录，以下命令会主动驱动机器人或注入故障。先按README设置RMUL_TEST_ARTIFACT_ROOT；受保护测试不得写入/tmp或WSL系统盘，默认不录bag。

```bash
ROS_DOMAIN_ID=178 python3 scripts/rmul2027/test_velocity_safety.py --rotation-mode additive --record-dir "$RMUL_TEST_ARTIFACT_ROOT/guard-additive-new"
ROS_DOMAIN_ID=179 python3 scripts/rmul2027/test_velocity_safety.py --rotation-mode heading --record-dir "$RMUL_TEST_ARTIFACT_ROOT/guard-heading-new"
bash scripts/rmul2027/run_point_time_contract.sh
python3 scripts/rmul2027/run_velocity_fault_sim.py --record-dir "$RMUL_TEST_ARTIFACT_ROOT/physical-fault-new"
bash scripts/rmul2027/build_cart_test_tools.sh
python3 scripts/rmul2027/run_cart_obstacle_test.py --record-dir "$RMUL_TEST_ARTIFACT_ROOT/parked-new" --scenario parked --params-file scripts/rmul2027/config/nav2_original_speed_early_brake.yaml --actor "$PWD/../../artifacts/rmul_2027_3v3/tools/obstacle_cart_actor" --dds-profile scripts/rmul2027/config/fastdds_local_64m.xml
```

完全封路沿用上一命令，换新record-dir、--scenario blocked，并加 --obstacle-model scripts/rmul2027/assets/test_barrier.sdf --obstacle-size 0.14 14 0.5。障碍为静态几何夹具，不是新增对手机器人、动态横穿或战术。它必须先进入雷达点云和代价地图；接触传感器先做正对照。自然ABORTED、实际停车且无正式接触后移除，双代价地图清除，再显式下发新目标验证恢复。测试取消不算自然失败通过。

## 点时间：C024/C025已完成快照链路验证

历史转换器输出time为秒、配置按US读取，存在已知失配；C025普通与fast入口已经采用后述已验证快照配置。Gazebo Sensors6的GPU雷达一次Render后以同一_now标记整帧，原字段无每点时间，因此为该模拟器提供instantaneous快照模式是基于实现的推断，不能照搬真实雷达。依据：[Gazebo原始实现](https://raw.githubusercontent.com/gazebosim/gz-sensors/ign-sensors6/src/GpuLidarSensor.cc)。

新增转换器point_time_mode，默认legacy_rolling不变；instantaneous输出零偏移。Point-LIO新增preprocess.point_time_is_valid=false；显式true才把全零时间视为合法provided-time，不再按方位角合成扫描时间，缺失字段及非法偏移明确拒绝。仅适用于VELO16输入分支，其他雷达保持原实现。

11项转换器／预处理测试及80项实际Kalman等价性检查通过。历史快照远点0.368m失败保留；C024修正批量信息更新的测量方差权重，并用实际外部连续48帧证实原IMU队列漏接后选择400。最终快照候选完成两次全新快速8/8与C025普通8/8、静态绕障和封路恢复，普通/fast入口现采用秒单位有效全零点偏移。max_imu_gap=0.1只在2027配置启用；真实历史缺口锁止后不再发布伪新定位，旧目标终止，需重启初始化。见C024/C025报告。

## 传输及边界

普通、fast与RViz入口共用configure_dds.sh，为本进程及子进程加载64MiB FastDDS共享内存配置，并记录hash；不改系统ROS或全局DDS。普通入口保持0.5m/s，fast仍是2.5m/s＋提前2m减速。RMUL2027_DDS_PROFILE可显式选择其他配置，但须另验收。64MiB是本机1.831MB点云的已验证设置，不是适用于所有机器的固定标定值。

完整结果见reports/C022-RESULTS.zh-CN.md，独立核查见reports/C023-AUDIT.zh-CN.md。WSLg显示与导航链路分开验收：当前后台窗口恢复调用经三次串行开关复核，手动恢复也有最小化正对照。其30秒轮询及当前会话通过，不代表跨Windows／WSL重启永久修复；历史共享内存EIO／COPY MODE保留。多项调用条件同时改变的试验只证明现组合可用，未证明某一单项为唯一故障原因。软件速度保护、Nav2自身时效检查和底盘独立看门狗也应分别描述，不能把已实现的fake_vel_transform保护误列为未实现。

C024另有桥接层与Gazebo插件独立0.5秒墙钟看门狗，以及Nav2 PoseFresh行为树条件；分别经13项物理故障注入与20项实际插件检查。它们覆盖上游停止或冻结时的不同故障范围，不能代替实车MCU保护。完整实现通过四份依赖补丁交付。

## C036/C038补充：严格停车与2D自身回波

普通验收保留原1秒四项速度/指令<0.03硬判定，再要求连续0.2秒稳定；固定2秒观测预算不能挽救原1秒失败。C034中心点1秒0.270823m/s仍记录失败，C036完整路线规划ABORTED不能因停车通过而算到点。C038仅在2D输入投影中按源时间戳TF过滤未膨胀车体footprint内点，不动安装/地图/规划/膨胀或定位；启动TF未就绪帧丢弃并计数，无真值输入。

接触夹具按每个collision1/collision2判真实对，显式探针对才排除，混合行不因probe标记被忽略。共享内存清理只针对本轮确证归属、基线排除、身份不变且内核lease确认无FD/mmap者；未知/活跃保留。各自契约与有限仿真结果见C036、C038报告，不将历史位姿停滞或规划失败因果写成已证明。
