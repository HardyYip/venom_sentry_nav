# C-020 原速度、提前减速与静态避障实验

用户要求恢复原速度验证，并测试哨兵对临时静态障碍的感知、绕行与停车。横穿、对手机器人开发及战术不在当前范围。所有程序只操作本场自动测试拥有的目标和进程，同一时间运行一套后台Gazebo。

当前已发布版本c4381c5的默认入口保持0.5m/s。新实验文件暂为本地新增，不等同已经提交或发布。下面的原速度和提前减速配置均为独立候选，实际是否通过以本轮报告和逐项结果为准。

## 速度对照

| 组别 | 最高线速度 | 接近最低速度 | 接近减速距离 | 仿真速度环 |
| --- | --- | --- | --- | --- |
| 已验收低速 | 0.5m/s | 0.1m/s | 1m | 2027当前PI配置 |
| 直接恢复原速度 | 2.5m/s | 0.5m/s | 1m | 2027当前PI配置 |
| 提前减速候选 | 2.5m/s | 0.5m/s | 2m | 2027当前PI配置 |
| 原底盘对照 | 2.5m/s | 0.5m/s | 1m | 原P[100,500]、I0、力上限[100,200] |

当前PI为P[1000,1000]、I[200,200]、力上限[200,400]。上述对照固定场景cad-flat-floor、原圆柱轮、IMU200Hz、定位参数、局部TF2修复、零自旋、足迹及0.35m保护。对照仅恢复速度或底盘参数组，不是恢复所有上游原始配置，因此不能据其失败断言上游原速度不可用。

在工程根目录，原速度实验入口：

```bash
bash scripts/rmul2027/start_rmul2027_original_speed.sh
```

该入口不自动发目标。严格原速度＋1m减速已出现停车超调，不能作为已通过配置默认使用。恢复已发布保守入口：

```bash
bash scripts/rmul2027/start_rmul2027_test.sh
```

自动复现以新的record-dir保存记录。先停止已有仿真，source ROS与当前workspace后执行：

```bash
source /opt/ros/humble/setup.bash
source ../../install/setup.bash
python3 scripts/rmul2027/run_timing_experiment.py \
  --record-dir /tmp/rmul2027-early-brake-new \
  --params-file scripts/rmul2027/config/nav2_original_speed_early_brake.yaml \
  --gz-world-path scripts/rmul2027/config/gz_world_test.yaml \
  --field-contact cad-flat-floor --render-mode gpu --last-case 07-occupied
```

直接恢复原速时，只换params-file为nav2_original_speed.yaml；原底盘对照在此基础上只换gz-world-path为gz_world_original_drive.yaml。每组从新出生点启动，任何目标失败即停止该组后续目标。运动目标要求180秒、估计／真值终点误差均小于0.35m并实际停车，不能只看SUCCEEDED。

## 临时静态障碍验证

测试夹具是一个与车体大小相近的几何障碍物，视觉主包络与碰撞一致，尺寸0.58×0.50×0.50m；没有新增导航机器人、传感器控制链或对手战术系统。障碍未写入静态地图，靠实时点云进入代价图。

先在新场景到达准备点map(1,-5.5)，正式目标为(5,-5.5)，障碍放在(3,-5.5)。control保持正式路线无障碍，parked加入障碍后走同路线。真值只用于测试评估及碰撞间距计算，不给导航定位或规划提供障碍坐标。

编译一次测试工具，需要当前Fortress开发依赖可由pkg-config找到：

```bash
bash scripts/rmul2027/build_cart_test_tools.sh
```

单组静态绕障：

```bash
python3 scripts/rmul2027/run_cart_obstacle_test.py \
  --scenario parked \
  --record-dir /tmp/rmul2027-static-obstacle-new \
  --params-file scripts/rmul2027/config/nav2_test.yaml \
  --actor ../../artifacts/rmul_2027_3v3/tools/obstacle_cart_actor
```

先做无障碍对照时，只把scenario换成control，并使用另一个新目录。若要测提前减速高速候选，只换params-file，其他条件保持。程序会自动启动、移动并结束自己拥有的仿真；不要同时手动发目标。当前自动夹具测试固定使用GPU渲染，软件渲染不在本轮结论范围内。

检查内容包括：静态地图该区域原本为空闲；障碍的点云返回及代价图更新；有可行通路时绕行并到点停车；真实足迹间距与接触记录；移走后代价图清除。接触传感器先通过远离比赛场地的正对照，避免把没有消息当作无碰撞。失败保护通过本组goal UUID请求取消，保留原始失败、路径、物理姿态与录包。

单个位置与尺寸通过只代表该场景能力，不能外推全场、所有障碍或比赛战术。全通道封堵应另测停车／明确失败，不应强求SUCCEEDED。

## 数据与恢复

本轮证据：`artifacts/rmul_2027_3v3/diagnostics/speed-obstacles-20261020-201657-C020`。记录包含实际参数和模型、库／资源校验、TF、点云／IMU、全局／局部代价图及updates、action隐藏话题、目标UUID、速度／真值、接触数据及失败帧。大型录包保留在工作区，不加入Git。

第一轮接触记录器话题／加载时序自检失败，以及初版测试模型视觉／碰撞包络不一致，均单独保留；最终静态结论以修正后模型的parked-02及后续同条件场次为准。已经执行过的横穿历史资料保留供审计，不是继续执行的任务或推荐验收路线。


## C-021：本地可选2.5m/s入口（2026-10-20）

新增 `bash scripts/rmul2027/start_rmul2027_fast.sh`：2.5m/s上限、提前2m减速，以及项目局部64MiB FastDDS共享内存。两次全新静态绕障和两次完整路线通过；普通 `_test.sh` 仍为0.5m/s，不静默替换。配置仅作用于该启动进程，不升级系统ROS。

发现的是本机1.8MB点云与默认0.5MiB共享内存容量不匹配导致的传输丢失；不是把导航算法改成另一套。验收脚本另增加发送目标前等待完整TF朝向链，运动／误差标准未变。结果包含一次修复前静态失败、一次测试工具TF中断和一次入口过早检查失败，均保留，详见 [C021结果](reports/C021-RESULTS.zh-CN.md)。

当前新增内容仅本地，尚未提交推送，不能认为GitHub旧HEAD已包含新入口。通过范围仅cad-flat-floor、GPU、静态场景与列明路线；点时间、指令超时保护和WSLg永久显示仍有未处理项。

自动静态复现使用B参数并加 `--dds-profile scripts/rmul2027/config/fastdds_local_64m.xml --record-pipeline`；完整路线同样加--dds-profile。先停止已有仿真，每次选择新的record-dir。需要RViz时，在该RViz启动命令前同样设置FASTRTPS_DEFAULT_PROFILES_FILE为该XML，保持单个RViz。
