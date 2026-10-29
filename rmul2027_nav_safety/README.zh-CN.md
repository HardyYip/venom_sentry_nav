# 等待/驻留后的 PoseFresh 时钟消费者

独立 BTv3 小包，处理 C062 低速实际复现的 source_future：PoseFresh 只在 tick 消费私有 /clock，空闲后读到旧钟，使新鲜 TF 被拒绝。

沿用 C061 已对照取证的持续执行器线程、互斥快照和析构 cancel/join。库名 rmul_pose_fresh_condition_bt_node，BT 节点名仍为 PoseFresh。原 TF 判定主体与旧源码逐字一致，timeout=0.5 / future_tolerance=0.1 保持。旧 pb_nav2_plugins dirty 源码和生产安装没有覆盖。

只在决策专用 C052 低速入口选择，不采用 fast 控制器/行为树，不声称历史 T4 或长期定位故障已解决。

tests/probe.cpp 与 tests/run_clock_contract.py 实际加载库，13tick 覆盖空闲新鲜定位、停钟、恢复、旧/未来/缺失 TF、常值时钟下戳停滞。构建和运行见相邻 rmul2027_decision/README.zh-CN.md；对照和本轮 provenance/contract 在 C062 唯一证据根。
