# 有界时序取证工具

该工具只订阅原生Gazebo clock、ROS clock、里程计/IMU/雷达头、TF和PoseFresh rosout，旁观/proc各相关进程的可见线程。它不发布速度、TF或时钟，不改导航和0.5秒保护。

健康运行只保存身份/最终摘要，原始20Hz样本保留在故障8MiB及预警4MiB内存环中。首次PoseFresh日志保存独立故障窗口；进入目标后的源progress超过0.35秒或观察循环空档，另存一次预警窗口。两者持续维护约3秒前窗和2秒后窗，早期可恢复预警不再抢占后续正式故障。0.35仅为取证触发值；导航仍按原0.5秒契约。故障文件最多32MiB，预警最多8MiB，总事件数据最多40MiB，整个测试仍走原50/40/30GiB、512MiB守护。

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

9项端到端覆盖健康流、冻TF、ROS钟停顿、正式日志、早预警后晚失败、目标结束后不再预警、观察器空档、SIGTERM、另建session的孤儿清理。工具始终自动冻结实际观察器二进制和自身Python源码，即使调用者未在identity映射中列出。
