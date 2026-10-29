# C066 交付总览：RMUL2027基础决策、跨场与IMU受控恢复

整理时间：2026-10-29T15:45:49.436832+08:00；执行者：Codex。发布目标：HardyYip/venom_sentry_nav 的 sim/rmul2027-3v3 分支。唯一主交接文件仍是用户指定的外部 read.md；本报告为仓库交付摘要。

## 验收状态

| 状态 | 交付及证据 |
| --- | --- |
| 已实现 | PB派生独立BT4决策；配置化TEST比赛状态/点位；等待、巡逻、条件撤回、恢复、结束/异常停车；Nav2单活动目标、去重、取消终态确认、有限重试与安全门禁。C063增加跨场最小覆盖；C064增加静止后重建自有导航树、epoch/真实GICP校验及最多两次受控恢复。 |
| 构建通过 | C062独立决策/BT4、安全overlay及BT3时钟插件；C064私有决策、安全和重定位诊断构建通过。生产安装未覆盖。 |
| 逻辑通过 | C062原Action stub22项、安全通道9项、原安全31项、时钟插件13tick；C064真实树及恢复逻辑CTest2/2（29项恢复用例）、原Action22项、新恢复Action7项、安全二进制16条检查记录。C066再次执行CTest2/2、18个Python语法及7个Shell语法检查，17份已验源码/运行库SHA256全部一致。 |
| 联调通过 | C062第09轮C052低速22项；C063最终两轮16段实际跨场到点、真实障碍移除/持续失败停车、绕行及采用后22项回归；C064第06/07轮各16项、共4次恢复后跨场到点，急停和恢复预算耗尽停车通过。 |
| 失败 | C062早期短路/跨场失败、C063扩大轮廓/权重候选与夹具失败、C064前5轮及入口/夹具失败原样保留。C065显示故障、演示脚本等待竞态和未完成阶段单列，不记作完整演示通过。 |
| 未运行/未验收 | 真车、正式2027裁判协议/规则阈值、完整对战、多车、追击射击、微小障碍/任意路线、动态物体主导的恢复、无限运行、fast避障、历史T4时序根因及新定位移植。 |

## 采用内容与安全边界

- BT4.6.2仅在独立决策进程；Nav2保持BT3.8.7，通过NavigateToPose Action连接。收到Topic发布应答或CancelGoal接受应答均不代表到点/取消完成。
- 原0.35m到点、180秒完整意图、触发后1仿真秒速度及指令全部分量低于0.03的门槛不变。点位、HP/热量/弹量均为明确标注TEST的配置，不是2027正式规则。
- 普通导航入口维持C052。C063专用start_low_speed.sh只覆盖RPLidar源最小高度0.06m与DWB时间积分0.05秒，轮廓、权重、padding及0.5m/s保持C052；跨场策略最多两次尝试，旧终态后停车等待3秒。
- C064仅由start_recovery.sh显式启用：独立停车与许可心跳、旧目标终态、原始IMU/激光静止检查、自有旧导航树退出、新epoch实际匹配/位姿/生命周期全部通过，才重新规划。Point-LIO原0.1秒缺口锁止保留，不能直接清除锁止继续积分。
- C062末尾定位故障为Nav2 ABORTED，不能写为取消成功。C064两轮10发送/10终态（4成功、6取消）是另一次版本与运行证据。
- C063最终16段最大真值误差0.151m、完整意图最长45.015516秒；C064四次恢复后到点最大真值误差0.136216m、完整意图最长52.744972秒、全部6次故障最大停车延迟0.555仿真秒。均为有限仿真资格。

## 复现与发布范围

先按[导航说明](../README.zh-CN.md)运行prepare_rmul2027.py恢复锁定依赖及补丁；再按[决策说明](../../../rmul2027_decision/README.zh-CN.md)隔离构建，受控恢复另见[恢复说明](../../../rmul2027_decision/recovery/README.zh-CN.md)。普通全工作区构建需跳过rmul2027_decision，避免BT4与Nav2 BT3混合。

small_gicp_relocalization的C064改动通过patches/manifest.json及small_gicp_relocalization.patch发布，不修改上游仓库和子模块gitlink。C066从锁定基准1df063ee6f67b592e1da47e2a21ded00321c0906干净恢复补丁，3份源码与已验版本逐字节一致。

本次只发布已采用决策、安全、跨场/恢复、必要测试和文档。原有未采用控制器/fast实验保持本地dirty；不reset/clean、不覆盖生产安装、不推上游或main。用户已取消继续演示编排，临时演示脚本归档后移出发布，不再打开RViz。可重建缓存可清理；必要回归测试、运行安装和全部原始失败证据保留。

## 原始证据索引

本机E盘证据不上传为仓库二进制或缓存：

- C062：/mnt/e/RMULArtifacts/pb2025_nav_ws-decision-C062-20261027；acceptance.json、final-audit.json、low-speed-attempt09-final事件与重放。
- C063：上述根内crossfield-C063；acceptance.json、final-audit.json及5份正式事件重放。
- C064：/mnt/e/RMULArtifacts/pb2025_nav_ws-imu-recovery-C064-20261028；acceptance.json、final-audit.json（45项）、replay-06/07.json、integration-07-code-sha256.json与各epoch运行库记录。
- C065：/mnt/e/RMULArtifacts/pb2025_nav_ws-rviz-decision-C065-20261028；演示原始失败、显示日志及已结束任务记录。
- C066：/mnt/e/RMULArtifacts/pb2025_nav_ws-publish-C066-20261028；补丁还原、最终逻辑检查、清理清单、提交及远端回读收据。

详细历史结果：[C062](../../../rmul2027_decision/RESULTS.zh-CN.md)、[C063](C063-RESULTS.zh-CN.md)、[C064](../../../rmul2027_decision/recovery/RESULTS.zh-CN.md)。历史报告中的“未提交”描述验收当时快照；后续GitHub交付以本轮提交和外部read.md的发布记录为准。

C066清理结果：删除313884511字节（299.34MiB）可重建缓存及自有临时文件，另保全3041252字节配置/日志；清理后17份已验源码/运行库指纹仍一致。WSL VHD未做压缩。
