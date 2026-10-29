# C063 2027跨场长路径修复与交付结果

执行：Codex。时间：2026-10-28T21:14:13.530994+08:00。本轮有限测试范围的实现、验证与审计完成；未提交或推送。

最终采用最小改动：专用start_low_speed入口在原C052配置上，只为两张图的RPLidar源设置min_obstacle_height=0.06，并启用固定时间积分，步长取原20Hz控制周期0.05秒。原矩形local/圆形global轮廓、padding、评分权重、控制器ELF、0.5m/s配置上限、地图、雷达挂载和定位保持。普通nav2_test.yaml及生产install未覆盖。

新增config/test_crossfield_policy.yaml是明确标记TEST的八段跨场策略，点位/阈值可配置，retry_delay=3秒、最多两次。旧短测试策略保留。只在旧目标终态确认后持续停车等待，再尝试；首次失败与后续成功分别记账。不是2027正式裁判协议或战术规则。

## 验收状态

| 项目 | 状态及实际证据 |
| --- | --- |
| 已实现 | 专用入口最小参数覆盖、新跨场TEST策略；测试工具支持显式私有overlay启动器，并在原严格准备门槛前等待生命周期激活。 |
| 构建通过 | Nav2真实库独立探针3个目标构建/CTest通过；导航/决策功能ELF未改，引用C062已通过构建及锁定SHA，没有重复全量构建。第三套几何诊断属于被拒绝候选，不作为采用依据。 |
| 逻辑/接口通过 | 沿用同版C062真实树/22项Action契约；新增3秒等待真实ROS Action对照3/3：持续拒绝、持续ABORTED最多两次并锁止，失败后安全等待恢复成功。 |
| 长路线联调通过 | 两次全新启动各8段实际到点，共16段；16次发送、16次成功、0次先ABORTED后恢复，取消0，单活动目标、无未结束所有权。每次到点原1秒停车通过；任何恢复失败后的1秒停车及等待期静止也通过。 |
| 真实障碍正负对照通过 | 最终同版physical-retry-minimal-01：障碍标入代价图、移走后第二次到点；持续障碍两次ABORTED后锁止，原1秒停车、无残留、接触0，最小名义净距0.499993m。 |
| 绕行/移除通过 | 最终同版minimal-obstacle-01：原实际传感器与两图检测、绕过0.58×0.50×0.50m物体、移除后清除，接触0，最小名义净距0.150772m≥原0.05m。 |
| 基础决策回归通过 | 采用专用入口后22/22：等待、真实到点/巡逻、撤退、恢复、裁判断流、定位许可、急停、结束及真实定位流中断停车。末定位故障本次终态为CANCELED，按实际记账。 |
| 失败 | 全部旧失败、被拒绝候选、脚本准备/取证错误保留在证据根；单轮成功不继承为其他配置通过。见下表。 |
| 未运行/未验收 | 真车、正式裁判协议/实战策略、追击射击、多车、fast/T4、新定位移植；微小障碍、原CAD坡道/立边、任意路径/无限运行；IMU缺口后的自动恢复。 |

长路线最大估计/真值终点误差0.143799/0.151000m，整意图首发送→最终成功（含失败及等待）最长45.015516秒。原0.35m/180秒/触发后1仿真秒vx/vy/w及命令全分量<0.03没有放宽。短时内部规划/控制失败仍可能发生，有限重试正确恢复与失败停车分别验收，不宣称零内部错误。

地图为项目官方2027 CAD保守栅格，沿用既有cad-flat-floor接触近似；没有替换为旧赛季地图。真值仅用于评估，从未注入定位/决策可信数据。

## 直接证据与取舍

- 源时间戳约3cm的2D地面端点曾进入lethal并切断道路；源高度过滤对这类端点生效，采用资格另由同版长路线和真实障碍测试确认。真实ObservationBuffer在变换后保留0.061/0.15/0.25m端点，实际实体障碍检测、绕行和移除为正对照。低于6cm的2D回波被排除，不继承微小障碍或实车资格；MID360与其他原硬门禁保留。
- 本机DWB距离步长模式从0.5m/s减到零，曾把模拟制动位移算为0；固定0.05秒后为0.1125m，首输出0.45m/s与20Hz、1m/s²一致，匀速轨迹保持。0.025秒半周期会改变输出加减速语义，已中止未采用。[Nav2 Humble对应源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_dwb_controller/dwb_plugins/src/standard_traj_generator.cpp)仅为参考，实际证据是本机1.1.20库探针。
- 2秒观测缓存中的短时目标占用，曾在两次0.3秒重试耗尽后才恢复；3秒停车等待覆盖该恢复机会。持续真实障碍正对照证明不是无限重试或绕过占用。
- 扩大local轮廓、global padding0.07、BaseObstacle4曾过部分长测，但实体绕行ABORTED，全部拒绝。静态几何探针通过不替代实际联调；最终恢复原C052几何与权重。
- 无支持证据的Navfn锁竞争/换算法、错误旋转、缺失静态地图假设均未实施。cmd_vel中间量处于稳定云台帧，桥接另作底盘变换；不能与底盘GT速度直接比较。GetCostmap中间无锁快照不作为实际规划图损坏依据，主要使用加锁costmap_raw发布和实际Navfn重放。

## 历史失败与未采用结果

| 运行 | 实际到点 | 发送/成功/ABORTED/取消 | 单轮结果（不表示采用） |
| --- | --- | --- | --- |
| adopted-decision-regression-01 | 3 | 8/3/0/5 | 单轮通过 |
| candidate-clearance-01 | 8 | 8/8/0/0 | 单轮通过 |
| candidate-clearance-02 | 4 | 6/4/2/0 | 失败，证据保留 |
| candidate-combined-01 | 4 | 7/4/3/0 | 失败，证据保留 |
| candidate-cycle-01 | 8 | 8/8/0/0 | 单轮通过 |
| candidate-cycle-02 | 7 | 9/7/2/0 | 失败，证据保留 |
| candidate-footprint-01 | 1 | 3/1/2/0 | 失败，证据保留 |
| candidate-ground-01 | 8 | 8/8/0/0 | 单轮通过 |
| candidate-ground-02 | 0 | 2/0/2/0 | 失败，证据保留 |
| candidate-minimal-01 | 8 | 8/8/0/0 | 单轮通过 |
| candidate-minimal-02 | 8 | 8/8/0/0 | 单轮通过 |
| candidate-path-01 | 1 | 3/1/2/0 | 失败，证据保留 |
| candidate-recovery-01 | 8 | 8/8/0/0 | 单轮通过 |
| candidate-recovery-02 | 8 | 9/8/1/0 | 单轮通过 |
| candidate-time-01 | 1 | 1/1/0/0 | 失败，证据保留 |
| focused-control-01 | 2 | 2/2/0/0 | 单轮通过 |
| long-route-01 | 8 | 8/8/0/0 | 单轮通过 |
| long-route-02 | 4 | 7/4/3/0 | 失败，证据保留 |
| physical-retry-01 | 0 | 0/0/0/0 | 失败，证据保留 |
| physical-retry-02 | 1 | 4/1/3/0 | 单轮通过 |
| physical-retry-minimal-01 | 1 | 4/1/3/0 | 单轮通过 |
| repro-original-01 | 1 | 1/1/0/0 | 单轮通过 |
| sensor-provenance-01 | 0 | 2/0/2/0 | 失败，证据保留 |
| sensor-stamped-01 | 8 | 11/8/0/3 | 单轮通过 |

focused-control-01实际两段，原JSON误沿用八段字段已在scope-correction.json更正，原文件保留。sensor-stamped-01有三次取消/恢复，不能记为八目标干净通过。stopped-retry-stub-01错从晚发日志取计时点，原Action终态后实际等待已≥3秒；修正观察点后02通过。physical-retry-01等待reason文本错误、绕行01/02私有库启动丢失，均在发目标前失败；绕行03实际ABORTED使扩大轮廓/权重候选被拒绝。不存在把这些失败覆盖或改写通过。

## 启动与交付边界

继续使用rmul2027_decision/README.zh-CN.md的三个终端流程。长回路将决策参数文件改为rmul2027_decision/config/test_crossfield_policy.yaml；先等待，确认导航/输入健康后切测试比赛running。只运行一套导航/仿真，遵守固定证据根与容量守护，不接管用户未知目标。

IMU数据中断仍保持原0.1秒保护锁止，需静止重新初始化，未实现自动恢复。C062历史末故障是ABORTED，本轮按实际终态分别记录，不混成取消成功。C062旧失败和资格未重写。

证据根：/mnt/e/RMULArtifacts/pb2025_nav_ws-decision-C062-20261027/crossfield-C063。索引acceptance.json、final-audit.json；两轮最终长测candidate-minimal-01/02、同版physical-retry-minimal-01、minimal-obstacle-01及adopted-decision-regression-01。此前其他候选不享有采用资格。
