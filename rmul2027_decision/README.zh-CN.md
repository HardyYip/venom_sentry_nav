# RMUL2027 基础决策（C062，含C063/C064扩展）

当前交付总览见 [C066](../scripts/rmul2027/reports/C066-DELIVERY.zh-CN.md)；[C063跨场结果](../scripts/rmul2027/reports/C063-RESULTS.zh-CN.md)与[C064恢复结果](recovery/RESULTS.zh-CN.md)分别记录通过范围及失败。

PB2025 的 RMUL 行为树是开发基础，COD2026 仅作策略参考。当前交付是测试比赛输入驱动的基础闭环，不代表已实现或核验 2027 裁判协议、赛季阈值和实车策略。

决策在独立进程中使用私有 BT4.6.2；Nav2 保持 BTv3.8.7、C052 普通控制器和 0.5m/s。专用入口另加载独立 BTv3 PoseFresh 时钟消费者及带决策心跳的速度安全节点。原普通入口、生产安装、历史 dirty 文件和全部失败证据保留。

## 行为与所有权

- 等待开赛不发目标；比赛中按配置驻留或巡逻。HP/弹量低或热量高时撤退，满足恢复阈值并稳定保持后继续原任务。
- patrol=false 到第一个任务点后驻守；patrol=true 仅在真实 Action SUCCEEDED 后驻留并轮换点位。
- 最多一个自有目标。重复输入不反复发目标；更换目标必须取消旧 UUID 并收到终态后再发。
- CancelGoal 接受应答只表示请求受理。无终态、取消拒绝/超时、接受超时或未知结果都会保持安全抑制，不把它们当作已取消。
- 拒绝、ABORTED、反馈/导航超时最多两次总尝试，重试延迟可配置。反馈/导航超时重试前也要完成取消与终态确认。接受超时有所有权歧义，直接锁止。
- 启动发现未知活动目标会锁止，不接管、不取消未知 UUID。决策实例有命名空间进程锁。
- SIGINT/SIGTERM 先抑制运动、取消自有目标并有界等候终态。SIGKILL/进程崩溃由独立速度节点心跳超时停车；死进程不能完成 Action 清理，重启前须核对遗留目标。启动保护不会抢占遗留的未知目标。

## 状态与安全门禁

config/test_policy.yaml 和 config/test_input.yaml 均为测试配置。HP 100/200、弹量 10/50、热量 80/40、坐标及驻留时间不是 2027 比赛规则。

| 命名空间内接口 | 类型 | 含义 |
| --- | --- | --- |
| decision/match | rmul2027_decision/msg/MatchState | 阶段、HP、弹量、热量、有效位、源戳、递增序号 |
| decision/estop | example_interfaces/msg/Bool | true 急停；须持续收到心跳 |
| decision/localization_trusted | 同上 | 外部可信许可；须持续收到心跳 |
| odometry、tf、tf_static | 原导航接口 | 独立检查有效姿态、TF、接收/源时间及戳推进 |
| navigate_to_pose | Nav2 Action | 仅管理自有 UUID |
| decision_stop | example_interfaces/msg/Bool | 20ms 心跳；true 抑制，独立于原 safety_stop |
| decision/events、decision/status | std_msgs/msg/String JSON | 单调时间、ROS 时间、UUID、目标、状态和故障 |
| decision/reset_fault | std_srvs/srv/Trigger | 仅等待阶段且无持有/待应答目标时复位执行故障 |

裁判默认接收/源时效 0.4 秒，外部状态心跳 0.4 秒，TF/里程计 0.5 秒，未来容差 0.1 秒。重复序号或源戳不刷新有效期。输入恢复后稳定 0.5 秒才继续；撤退恢复使用分离阈值及保持时间。比赛结束锁止到本次进程退出，新比赛须确认无残留目标后重启。

C062原版后端没有独立定位质量/重定位 epoch 协议。C064新增显式受控恢复协议，须使用[恢复入口](recovery/README.zh-CN.md)，并不代表通用实车定位质量资格。外部许可、有效姿态、源/接收时间及 TF 推进组成当前可信门禁，不保证能检测“冻结位姿但持续伪造新戳”。接实车时须由真实定位状态适配器提供许可，不能用仿真真值。本轮真值仅用于独立评估到点和停车。

速度节点新参数 decision_stop_timeout=0.3 只在专用入口启用。原急停与决策抑制按 OR 处理，任一路解除都不能覆盖另一来源；缺失/过期心跳也停车，恢复后必须有新命令。默认 0 保持旧入口行为。

## 独立构建

在 WSL 中，整个任务固定同一个证据根，不靠换根扩大预算：

    export RMUL_TEST_ARTIFACT_ROOT=/mnt/e/RMULArtifacts/pb2025_nav_ws-decision-C062-20261027
    bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/rmul2027_decision/tools/build_decision.sh

脚本锁定 BT4.6.2 commit，分别安装到 artifacts/rmul_2027_3v3/decision-C062 的 bt4、install、safety-install、nav-safety-install；不修改 /opt/ros 或现有工作区 install。RMUL_DECISION_BUILD_ROOT 可指定私有前缀，所有入口须用相同值。依赖锁见 dependencies.lock.json，PB 参考 XML 与许可证位于 reference。 BT依赖显式禁用ament发现（CMAKE_DISABLE_FIND_PACKAGE_ament_cmake=ON），避免source ROS后自动切换构建模式；决策ROS包仍用ament。

构建/测试/运行均须由 tools/guarded_run.sh 或既有守护包装。保持 D/产物盘 50GiB 开始、40 预警、30 停止，单轮 512MiB/1800 秒、会话 4GiB、宿主增量 2GiB。无整段 bag。开始前检查依赖/构建/证据合计并预留一轮预算。

## 手动测试闭环

确认无另一 AI、用户手动目标、其他导航或仿真后，三个 WSL 终端使用同一证据根。缺失输入时决策保持停车。

终端 1，仅一套专用低速导航：

    export RMUL_TEST_ARTIFACT_ROOT=/mnt/e/RMULArtifacts/pb2025_nav_ws-decision-C062-20261027
    bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/rmul2027_decision/tools/start_low_speed.sh

终端 2，独立决策：

    export RMUL_TEST_ARTIFACT_ROOT=/mnt/e/RMULArtifacts/pb2025_nav_ws-decision-C062-20261027
    bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/rmul2027_decision/tools/start_decision.sh --ros-args --params-file /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/rmul2027_decision/config/test_policy.yaml -r __ns:=/red_standard_robot1 -r /tf:=tf -r /tf_static:=tf_static -p event_file:=/mnt/e/RMULArtifacts/pb2025_nav_ws-decision-C062-20261027/manual-events.jsonl

终端 3，测试输入。先将 config/test_input.yaml 复制到证据根的 manual-input.yaml：

    source /opt/ros/humble/setup.bash
    source /home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/decision-C062/install/local_setup.bash
    python3 /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/rmul2027_decision/tools/test_match_input.py --input-file /mnt/e/RMULArtifacts/pb2025_nav_ws-decision-C062-20261027/manual-input.yaml

初始为 waiting、急停 true、定位许可 false。确认导航实际健康后，编辑测试文件为 estop=false、localization_trusted=true，再切 phase=running。改变 hp/ammo/heat 验证撤退与恢复；phase=finished 验证结束。工具持续重读文件，退出或读入失败会导致许可/心跳失效。测试许可不能替代真车定位质量。

正常结束先设置 finished 并观察 UUID 终态和实际全零，再关闭决策/导航。SIGINT/SIGTERM 也会有界取消；取消失败保持锁止并记录。只管理本次自有进程和目标。

## 验证与事件回放

纯逻辑使用真实 BT 树；22项真实 ROS Action stub 包括拒绝、ABORTED、反馈/导航超时、取消拒绝/无终态/应答延迟、迟到接受、源戳重放/未来戳、断流、定位/急停失效与恢复、未知活动目标及退出取消。stub 须使用独立 ROS_DOMAIN_ID，并外包容量守护。

- tests/action_stub_contract.py：真实 ROS stub，不启动 Gazebo。
- tests/safety_gate_contract.py：实际速度安全二进制。
- ../scripts/rmul2027/test_velocity_safety.py：新安全通道默认关闭时原契约回归。
- ../rmul2027_nav_safety/tests/run_clock_contract.py：实际 BTv3 插件13tick。
- tests/low_speed_integration.py：独占启动/结束一套 Gazebo/Nav2，不能与手测并行。

各脚本接收新建输出目录；具体参数可用 --help。离线事件回放审计：

    python3 /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/rmul2027_decision/tools/replay_trace.py <decision-events.jsonl> --output <replay-audit.json>

该脚本检查目标所有权、UUID、取消与终态先后及未解决目标，不重跑 ROS 或物理仿真。到点必须同时有 Action SUCCEEDED、估计/真值误差均不超过 0.35m，并在原180秒内完成。停车使用原触发后1仿真秒的 vx/vy/w 与指令均低于0.03。保护停止、取消、失败及正常到点分别记录。

本轮结果、失败和未运行范围见 RESULTS.zh-CN.md；唯一交接文件仍为用户指定 read.md。

## C063 2027地图跨场扩展（有限验收）

专用start_low_speed在原C052参数基础上加入0.06m RPLidar源地面过滤和与20Hz控制周期一致的0.05秒DWB积分。轮廓、padding、评分权重和速度保持原C052；普通基配置和生产install不改。扩大局部轮廓/权重的旧候选被绕行回归拒绝，不要采用证据目录里的旧候选yaml。

长回路可在上述手动闭环的决策命令中使用config/test_crossfield_policy.yaml。它是TEST策略，八个跨场路点、驻留4秒、旧目标终态后停车等待3秒再做已有第二次尝试；保持最多两次、原180秒/0.35m/1秒停车门槛。旧短策略test_policy.yaml仍保留。正式赛季阈值/协议、真车和微小障碍不在本轮资格内。

实际移除障碍恢复、持续障碍有限失败停车、实体绕行/清除及采用后的22项回归通过。详细指标和全部失败见../scripts/rmul2027/reports/C063-RESULTS.zh-CN.md（从本包回到仓库后为scripts/rmul2027/reports）。IMU缺口仍需静止重新初始化；长测成功不是任意路径/无限运行保证。


## C067 时序修复后的入口

完整验收、失败与边界见[结果报告](../scripts/rmul2027/reports/C067-RESULTS.zh-CN.md)。原策略/BT和普通不自动恢复行为保持；项目DDS默认改为UDPv4/4MiB，fast入口保留旧传输。显式恢复入口还检查TF源时间/墙钟推进，并独立消费IMU/源钟，原门槛不变。

先设置固定RMUL_TEST_ARTIFACT_ROOT，再执行tools/build_timing.sh独立构建。现有C062基础及C064安全/重定位私有安装保留；C067决策安装在artifacts/rmul_2027_3v3/timing-C067。恢复启动说明见[恢复说明](recovery/README.zh-CN.md)。本轮没有新增正式赛季策略、真车或fast资格。
