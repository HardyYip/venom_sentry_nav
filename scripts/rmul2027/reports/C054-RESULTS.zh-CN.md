# C054 只读时序取证交付与剩余边界

Codex，2026-10-24T14:21:24.913642+08:00。本轮完成取证工具及独立验证，**未完成历史T4根因定位或修复**。普通0.5m/s已发布导航与C052固定12/同速移动3有限验收保持；未采用DeepSeek被拒候选。

## 已完成

- 新受守护入口run_timing_capture.py；独立订阅native clock/IMU/里程计、ROS时钟/传感器头/TF/PoseFresh日志，旁观相关进程可见线程的/proc资源。无导航控制输入。
- 故障与预警各维护约3秒前窗、2秒后窗；早预警恢复不抢占后续正式故障窗口。内存环8+4MiB，故障32MiB、预警8MiB上限；仍受原50/40/30GiB、单轮512MiB守护。健康流不落整段原始数据。
- 目标产生原验证器的终态stop-state后解除源停顿预警；正式PoseFresh日志仍捕获。队列丢样、观察空档、截断、后窗不足、资源线程上限均可见。窗口完整不等于因果已定。
- 自动冻结实际观察器二进制及自身源码，保留用户声明身份和真实launcher runtime来源；自有进程按独立继承token及PID启动时间清理，覆盖另建session孤儿。
- 最终13项离线契约、9项可移植真实ROS/Ignition消息及生命周期契约通过。独立规范patch应用到空目录逐文件SHA一致，ROS-only观察器和夹具构建通过；最后只改入口/测试脚本，编译源/CMake/采集模块与已构建字节一致，最终包再做13项及9项验证、真实入口子命令参数透传检查。测试夹具必须在私有域91及独立IGN partition，并在其他域拒绝启动。
- DeepSeek候选可重建build/install按SHA清理42269314字节；源码、实际库、失败和原测试日志保留。本轮其他可重建清理另见cleanup-ledger.json。

## 两轮真实集成：失败和通过均保留

| 运行 | 实际结果 | 可作何结论 |
| --- | --- | --- |
| integration-route | 准备导航native6；PoseFresh stamp_stalled，source29.2/now29.499/age0.299；原1秒停车通过；正式移动未运行 | 同名保护再次出现；v1唯一窗口被更早可恢复预警占用，正式错误不在窗口内，不能定位根因 |
| v2-integration-route | 准备和移动遭遇通过；名义gap0.096493m；原停车通过；无PoseFresh失败 | 有限集成通过；预警发生于最后目标trace之后3.637秒，是停机阶段，不作为活动导航故障证据 |

两轮真实startup/end全部条目一致、目标UUID时段被实际快照包围，原1秒四项速度小于0.03、2秒预算、0.2秒稳定门槛未改。详细逐目标原始审计在independent-integration-audit.json。第二轮之后仅对采集阶段门控/自证/入口作定向修正，不重跑导航来抹去首次失败；最终工具新增边界由对应契约验证。原导航/定位/PoseFresh源码、库、速度和默认均未修改。

早期原始窗口中native clock正常而ROS/定位短滞后，只能描述该早期窗口，不能外推到更晚正式错误。第二轮停机窗口中native传感器继续而ROS流停下，也不能宣布桥接是历史根因。1Hz线程资源是快照，不是阻塞栈或完整调度证明。

第一版夹具ROS C++接口编译失败、第一轮单窗口缺口、独立移植测试脚本变量冲突及入口透传边界均保留记录并修正；没有把这些失败写成通过。

## 交付与未完成项

工具源及可移植测试位于scripts/rmul2027/diagnostics；COLCON_IGNORE确保不加入生产构建。受守护入口scripts/rmul2027/run_timing_capture.py，使用说明diagnostics/README.zh-CN.md。规范patch SHA256：6e78d167cc150117c107b4df8d1349320d781c14dc28c24096e7eb167d6f2634。准确依赖版本在tested-diagnostic-packages.tsv。

原始证据目录：/mnt/e/RMULArtifacts/pb2025_nav_ws/codex-trigger-capture-C054-20261024-133406
关键文件：final-source-verification.json、independent-restoration.json（首移植尝试含失败，不单独当最终通过）、final-portable-live/live-tests.json、independent-integration-audit.json、final-production-check.json、runtime-copy-manifest.json、v1-window-limitation.json。必要运行日志已按SHA复制E盘，原位置也保留；无整段bag、无RViz。

仍未完成：
1. T4历史根因和对应实际修复/回归：未取得活动导航正式失败同窗的native/ROS/TF证据，不能合理决定唯一修复环节。
2. 完整人工往返：未运行，历史完整路径未定义/记录；自动末段/路线不能冒充人工序列。
3. fast新避障：未运行，现有2.5m/s配置仍选原Omni控制器，不具备普通CollisionAware的验收依据，未改快入口默认。

唯一下一证据目标：在后续明确的活动导航复现中，用已验证的正式失败窗口定位最先中断环节；保留原保护，不再无限盲跑，不宣称全部任务完成。
