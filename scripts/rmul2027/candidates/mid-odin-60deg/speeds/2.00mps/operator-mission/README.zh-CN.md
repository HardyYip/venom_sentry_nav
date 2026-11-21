# 2.00 m/s 操作员返航仿真验收

本包在一个持续运行的 Gazebo 场景中驱动 MID360＋Odin1 模拟车。测试节点直接持有 Nav2 Action 目标，不注入到点回执。2.00 m/s 是平面速度上限。

将本目录放在既有仿真环境的 /evidence 对应宿主目录下，以 bash run.bash 启动。运行依赖原隔离器 /tmp/rmul-jammy/sim_shell.bash、GPU 渲染、既有 ROS 工作区，以及 /evidence/c072-speed200-from-qualified150-20261121 下的配置、两个已编译的安装前缀和 DDS 配置。导航基线提交为 0a92ab994720e1dbfac18e4d8b73c33842d2228c。运行会创建独立的 trial 目录。

mission.py 负责发送及跟踪目标；recall_client.py 代表测试操作员，观察到第二段已运动后调用 operator_recall 服务一次。任务依次到达 A、前往 B、取消 B、停稳、回站、恢复 B、再回 A。每次到点均检查 Action SUCCEEDED、底盘真值距目标不超过 0.25 m，以及实际速度和指令在连续两秒内均低于 0.03；取消后要求原 UUID 获得 CANCELED，连续一秒停稳后才发回站目标。坐标 A=(2.1,-5.5)、B=(5,-5.5)、站点=(1,-5.5)，单位为米。

本流程验证操作员返航请求。触发来源和任务节点独立于历史比赛行为树。trajectory.jsonl 保存 Gazebo 真值，events.json 保存目标 UUID、终态和到点误差，ready.log 保存就绪检查。
