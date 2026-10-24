# C056：取证终态门控和真实引擎线程覆盖

Codex，2026-10-24。T4历史stamp_stalled仍未定位或修复；本次只修正采集器的两个可复核缺口，生产导航、PoseFresh、参数、默认和原安全门槛均未改。

占用目标按预期拒绝时，原验证器记录invalid_result/status6但不生成stop-state，旧采集器继续布防，曾在suite结束1.071229秒后误存停机预警。现仅在尾部最多64KiB内找到与ownership UUID匹配的goal_accepted及其后导航拒绝终态时解除预警；缺失、半写、不同UUID继续保守布防。正式PoseFresh日志始终触发。

原资源过滤未识别实际进程标题ign gazebo，遗漏真实引擎线程。现按实际进程标题或可执行文件名识别；启动shell与携带观察器参数的父进程不冒充引擎/观察器。32进程、每进程128线程上限与缺测标记保持，1Hz快照不能独立证明阻塞或调度因果。

## 验证

规范patch从HEAD独立恢复，三份Python源码SHA与工作副本一致，18项离线及10项隔离真实ROS/IGN消息和生命周期契约全过。旧终态实现的两个反例失败，修正后通过。实际引擎线程覆盖另有原始审计。18/10仅验证取证工具，不是T4故障修复回归。

四次无注入运行共有47个自有目标：45成功、1次其他导航ABORTED、1次预期占用拒绝。真实startup/end的14项身份及目标UUID时段包围均核对，运动目标的原1秒停车和稳定确认均通过；占用拒绝独立核查零命令与微小位移。

| 运行 | 自有目标 | 结果 |
| --- | ---: | --- |
| 准备+遭遇 | 2 | 通过，净距0.124179m |
| 普通完整8项 | 7 | suite通过；唯一预警在结束后1.071229秒，属停机 |
| 有界持续路线 | 2 | 01通过；02进展失败/无有效轨迹/SpinCoverage后ABORTED，后续未运行 |
| 有界空旷走廊 | 36 | 36/36通过，正式错误及预警0 |

四次正式PoseFresh失败均为0。这些健康窗口不能替代历史正式故障同窗，route-03也不能混为T4。历史日志中邻近的odometry_receive_timeout只能证明接收超时同时出现，不能唯一归因LIO、桥接、DDS、调度或TF缓存。

## 证据

完整原始产物：/mnt/e/RMULArtifacts/pb2025_nav_ws/codex-timing-C056-20261024-150851。

- independent-runtime-audit.json：真实运行身份、目标时段和原停车。
- final-source-verification.json：三源码SHA、规范patch SHA及18/10结果。
- diagnostics-C056.patch、restored-diagnostics：可独立恢复的三文件修正。
- gazebo-resource-filter-audit.json、route-04/audit/actual-engine-coverage.json：实际进程及线程覆盖。
- historical-failure-index.json：历史日志索引，含注入/停机记录，不能把所有条目都算T4。

用户已确认C055自由手测效果，未再强加指定人工A/B路线。当前主线为T4取证、证据支持的单项修复及原门槛回归；之后才进行fast新避障。fast仍使用旧Omni控制器，不能继承普通0.5m/s的C052验收。
