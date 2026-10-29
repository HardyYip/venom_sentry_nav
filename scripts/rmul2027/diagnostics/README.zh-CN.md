# 有界时序取证工具

该工具只订阅原生Gazebo世界/全局clock、ROS clock、里程计/IMU/雷达头、TF和PoseFresh rosout，旁观/proc各相关进程的可见线程。它不发布速度、TF或时钟，不改导航和0.5秒保护。

健康运行只保存身份/最终摘要，原始20Hz样本保留在故障8MiB及预警4MiB内存环中。首次PoseFresh日志保存独立故障窗口；进入目标后的源progress超过0.35秒或观察循环空档，另存一次预警窗口。两者持续维护约3秒前窗和2秒后窗，早期可恢复预警不再抢占后续正式故障。0.35仅为取证触发值；导航仍按原0.5秒契约。故障文件最多32MiB，预警最多8MiB，另有独立4MiB内存/8MiB文件的时钟偏差窗口，总事件文件最多48MiB、内存环最多16MiB，整个测试仍走原50/40/30GiB、512MiB守护。

缺测、队列丢样、字段溢出、文件截断和后窗不足分别记录；无事件不等于历史问题已修。原生clock接收停顿也不能区分发布、传输及观察器调度。线程资源为1Hz快照，每进程最多128线程、最多32进程，缺失/上限显式标记。TF收包时间为观察器时间；不宣称这是生产BT实际回调时刻。

构建依赖：Ubuntu22.04/ROS Humble，ament_cmake、rclcpp、rcl_interfaces、rosgraph_msgs、nav_msgs、sensor_msgs、tf2_msgs、nlohmann-json3-dev，以及pkg-config的ignition-transport11和ignition-msgs8。独立CMake构建，不改生产工作区安装库；构建/测试也须置于原storage_guard下，产物目录在E盘。

构建形式：
    cmake -S scripts/rmul2027/diagnostics -B <E盘本轮目录>/observer-build
    cmake --build <E盘本轮目录>/observer-build -j2

在已source ROS/工作区和configure_dds.sh的同一个环境使用受守护入口：
    python3 scripts/rmul2027/run_timing_capture.py --record-dir <E盘新目录>/capture --observer <本轮目录>/observer-build/timing_chain_observer --identity <身份路径JSON> --arm-root <本轮导航case目录> --seconds 480 -- <导航测试命令及参数>

identity JSON是字段名到实际文件绝对路径的映射，最多64项。启动/结束保存SHA和字节，真实launcher runtime快照从arm-root下session.json的navigation_log定位并复制；不得以自身preflight代替真实运行来源。arm-root用本次全新case目录，检测到尚无终态stop-state快照的目标ownership后才启用源停顿预触发；已完成目标会解除预警，防止停机流量被误计为活动导航故障，PoseFresh日志始终触发。保持ROS_DOMAIN_ID、DDS profile及IGN_PARTITION与目标进程一致。没有目标导航命令时可在有限时长内旁观，但仍须原守护。

支持SIGINT/SIGTERM。工具清理自己启动的进程组，按独立继承token和PID启动时间核对，包括命令父进程已退出或子进程另建session的情形；不扫描并终止其他仿真。容量守护继续独立按run token处理自有进程。只有观察器结束、采集不足或受控中断时，摘要明确为incomplete。事件完整不代表因果已定。

验证：test_timing_capture.py覆盖窗口边界、内存/文件上限、无事件不落全流、缺测、NaN、日志触发及线程字段；另用真实ROS+Ignition消息夹具和进程生命周期检查端到端行为。确切结果和失败尝试见C054报告。夹具注入不作为历史故障复现或导航避障通过。


可移植的隔离端到端契约在tests目录。其C++夹具会发布合成时钟/TF，必须使用ROS_DOMAIN_ID=91、IGN_PARTITION=codex_c054_fixture、原容量guard，且采用无共享内存的回环DDS profile。夹具在任何其他域中拒绝启动，避免影响导航。先独立CMake构建tests中的fixture，再在同一个guard内调用：
    python3 diagnostics/tests/run_live_contract.py --record-dir <E盘新目录>/contracts --observer <观察器可执行文件> --fixture <fixture可执行文件> --identity <身份路径JSON>

10项端到端覆盖健康流、冻TF、ROS钟停顿、正式日志、早预警后晚失败、目标结束后不再预警、占用拒绝终态、观察器空档、SIGTERM、另建session的孤儿清理。工具始终自动冻结实际观察器二进制和自身Python源码，即使调用者未在identity映射中列出。


## C056：终态与进程覆盖补充

占用目标可能按预期产生invalid_result/status6而不写stop-state.json。取证布防现在还读取同目录verify.jsonl尾部最多64KiB，只有goal_accepted的UUID与本次ownership一致、随后出现该导航拒绝终态时才解除预警布防。未知、缺失、截断或无匹配UUID继续保守布防。正式PoseFresh日志触发不依赖布防状态，因此不会因为目标结束而丢弃正式错误。

资源采样按实际可执行进程识别，覆盖进程标题为ign gazebo的引擎，不再用启动shell代替引擎线程。原每进程128线程、32进程上限和缺测标记保留。资源计数仍是1Hz快照，不是阻塞栈或连续调度证明。

对应回归增加真实占用拒绝终态、不同UUID、半写记录及实际Gazebo标题反例；隔离端到端增加rejected_goal场景。普通导航、PoseFresh的0.5秒墙钟保护、IMU缺口锁止和停车门槛均未修改。修正采集缺口不代表历史stamp_stalled根因已确定；本轮结果见[本轮报告](../reports/C056-RESULTS.zh-CN.md)。


## C067：独立时钟链与输入缺口窗口

观察器同时读取世界 /world/default/clock、桥接真实输入 /clock 及ROS /clock。时钟偏差窗口独立于预期TF故障窗口；在样本和收包新鲜条件满足时，以0.1秒偏差触发，保存3秒前窗和2秒后窗。该值只用于取证，不修改生产保护。摘要明确偏差发生在世界到全局或原生到ROS阶段；它不自行确定因果，也不声称当时存在活动目标。

原生IMU与ROS IMU分别保留最近最多16个大于0.09秒的源戳间隔，包含源戳及观察器收包间隔，随有界窗口和最终摘要保存。ROS观察器QoS深度1可能漏采，不能把观察器的缺口直接当成生产输入丢包；必须和原生源、生产监督器故障快照、实际后端日志对照。新的监督器快照同时记录输入无效原因、最近源戳间隔与收包间隔。源戳缺口阈值的取证记录不改变原0.1秒IMU锁止。
