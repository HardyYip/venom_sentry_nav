# C064 IMU断流后的受控恢复

当前验收结果见同目录RESULTS.zh-CN.md；未标联调通过前，不把代码存在当作恢复交付。

本入口沿用C063最小覆盖、C052低速控制器、Point-LIO和small_gicp算法。生产安装不覆盖；BT4仍只在独立决策进程，Nav2仍为BT3。只允许一套Gazebo/导航。

## 恢复顺序

1. 原IMU缺口阈值仍为0.1秒。原后端永久锁止保持，不直接清除标志或跨缺口积分。
2. 独立发布recovery/stop=true、recovery/permit=false。底盘OR停车通道与决策AND许可都有0.3秒失心跳保护，不能覆盖原急停、裁判、信任或定位门禁；恢复后仍需要新命令。
3. 等待决策报告无自有目标且无故障，并检查Action状态无活动目标。取消应答不等于终态；ABORTED和CANCELED分别记账。5秒无终态就锁止，不重启。
4. 原始IMU连续、激光点云可观测且静止至少2秒，实际输出命令持续为零，外部门禁健康后，重启监督器自己启动的导航进程树。Gazebo保留。全部旧LIO、重定位、terrain、代价图及控制轨迹一起退出；退出未确认就保持停车。
5. 每次重启增加epoch。它是本轮定位的“代次编号”，避免旧TF、旧服务响应和旧匹配结果冒充新初始化。Point-LIO从静止重新初始化。
6. 用故障前的估计map→gimbal_yaw作为种子，经新鲜odom上的initialpose接口换算。此处是稳定云台坐标，不能拿底盘朝向代替。初始种子仅缩小搜索范围，其确认不代表定位成功。
7. 必须收到本epoch、种子之后的4次实际GICP收敛结果；检查点数/内点、平均误差和信息矩阵，确认实际TF/odom新鲜、底盘位姿稳定且map→gimbal_yaw未偏离同一稳定云台种子超过0.35m/0.15rad；独立静止、外部门禁、新Nav2生命周期active均通过后才放行。底盘约3.1rad/s自旋停车时可能转过1rad以上，此真实转角不能拿来做稳定云台种子偏差。旧生命周期客户端销毁，查询超时0.8秒后重建，不能一直等已退出的服务器。
8. 决策重新发送未完成任务的新Action目标，旧目标不复活。每次启动最多2次自动恢复；第3次故障、重启失败/60秒超时、新定位30秒不通过、静止/外部条件30秒不通过等都锁止停车。

首次启动有90秒准备上限；其后每次定位校验30秒。原始源时间和单调接收时间分别检查；不是只看消息在到达。

## 本轮静止/质量设置

这些是明确的仿真TEST设置，未作真车资格：原始激光0.12m抽样、至多4000点、0.5–12m范围，至少200个有效匹配，点到平面的6维可观测性比例至少1e-4；相对锚帧平移/转动各不超过0.015m/rad、残差RMSE不超过0.02m。单一地平面/空点云不允许证明静止。IMU角速度<0.03rad/s，重力模长与9.81差<0.2m/s²，逐轴加速度标准差<0.1m/s²。点云/IMU取自实际传感器话题，不能用冻结后的LIO速度证明停车。

GICP门槛：至少200点/内点、内点率≥0.5、平均GICP代价≤0.5、Hessian特征值比≥1e-6。GICP代价不是米；Hessian表达本次匹配对位姿约束是否充足。这些量来自实际align调用，TF发布计时器不产生“匹配成功”。

## 构建与自动验收

已有ROS Humble、numpy/scipy、sensor_msgs_py及本机锁定依赖。复用C062 BT4和时钟插件、本地deps/small_gicp和TF2核心；不拉取新依赖。

构建：

    bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/rmul2027_decision/tools/build_recovery.sh

单套自动验收（先核实没有其他导航、测试或输入馈送）：

    bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/rmul2027_decision/tools/test_recovery.sh --record-dir /mnt/e/RMULArtifacts/pb2025_nav_ws-imu-recovery-C064-20261028/manual-unique-name

测试会生成明确的TEST比赛输入，转发原始IMU到recovery/test_imu并让定位与监督器订阅同一输入，在2027地图做真实输入中断及后端暂停/缺口、1秒停车、恢复后跨场到点、初始化时急停、恢复预算耗尽停车。测试订阅真值做独立验收，运行门禁不订阅真值。目录必须新建；不要与手动TEST输入同时运行。

## 手动启动

设置RMUL_TEST_ARTIFACT_ROOT为上述固定证据根。导航用tools/start_recovery.sh；决策设置RMUL_DECISION_BUILD_ROOT为私有imu-recovery-C064，启动tools/start_decision.sh并显式传入ROS参数recovery_permit_timeout:=0.3。需要持续TEST裁判/急停/信任心跳，可用tools/test_match_input.py --input-file 指向独立测试输入文件。普通start_low_speed.sh不默认启用恢复，也不切换既有采用资格。

底盘recovery_stop_timeout和决策recovery_permit_timeout默认都是0，以保持已有普通入口行为。C064入口底盘显式设0.3；监督器还核实决策报告0.3，缺少正确配置不会放行。故障FAILED不提供自动清除按钮；需结束自有测试并重新启动，不能反复重置凑通过。

真车、正式裁判协议、场地对称误匹配、动态物体占主导时的自动恢复、fast及新定位移植仍另行验收；T4最新有限范围见下文C067。失败记录保留，不能以静止判定取代真实急停或物理停车测试。


## C067 当前已验时序入口

结果见[C067报告](../../scripts/rmul2027/reports/C067-RESULTS.zh-CN.md)。除原IMU/odom外，TF源戳过期/停滞也触发同一受控协议。IMU和真实ROS源钟由同进程独立Node/执行线程接收；400条有界队列不改变0.1秒源时效/缺口检查。主流程只读不可变快照，失去实际输入或心跳仍停车。固定release_verified记录释放瞬间实际4次匹配与门禁；后来滚动quality_count变小不代表当时未通过，也不能拿历史释放证据替代当前permit。

已用满两次恢复后仍先确认旧Action终态，再立即FAILED；不再等待只为重启服务的静止检查。任何允许重启的情况仍必须经过原始输入、2秒独立静止、外部门禁、真实新epoch质量和生命周期检查。出生区0.2m质量不足超时案例保留为安全锁止，不保证所有点位都自动恢复。

在同一个固定容量会话中设置：

    export RMUL_TEST_ARTIFACT_ROOT=/mnt/e/RMULArtifacts/pb2025_nav_ws-timing-C067-20261029
    export RMUL_DECISION_BUILD_ROOT=/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/timing-C067
    export RMUL_NAV_BASE_BUILD_ROOT=/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/decision-C062
    export RMUL_RECOVERY_BUILD_ROOT=/home/hardy/pb2025_nav_ws/artifacts/rmul_2027_3v3/imu-recovery-C064
    bash /home/hardy/pb2025_nav_ws/src/pb2025_sentry_nav/rmul2027_decision/tools/build_timing.sh

手动导航仍用tools/start_recovery.sh；决策用tools/start_decision.sh，显式传recovery_permit_timeout:=0.3及所需TEST策略/事件路径。只能运行一套导航及一份TEST输入。

自动时序验收：tools/test_timing.sh --record-dir <固定根下新目录> --fault-min-distance 1.2；持续质量缺失负向追加--mode persistent。原IMU完整回归用tools/test_recovery.sh；它保留原消息与真实停流/后端暂停注入，转发Node独立于记录和TEST心跳。先确认没有其他导航或输入馈送；目录必须全新。真值只用于测试触发/验收，不参与产品恢复门禁。普通入口保持不自动恢复。
