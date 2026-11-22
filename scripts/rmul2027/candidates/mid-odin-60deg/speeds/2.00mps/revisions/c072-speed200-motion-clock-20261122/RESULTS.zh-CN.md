# 2.00 m/s 速度反馈修复候选

本分支从已验收的 `b4a4657` 派生。候选用于仿真复查，整轮验收尚未通过。

局部规划器输出通过原轨迹检查的目标速度。速度平滑器从上一拍指令开始，按 ROS 时间限幅，每轴加减速 ±1.75 m/s²，平面速度上限 2.00 m/s。里程计用源位姿时间戳差分，减少消息处理时序造成的测速抖动。

七条长路线均到点并停稳，墙体、箱体接触均为零，Gazebo 底盘峰速 1.999 m/s。双向源时钟对照的测速平均绝对误差由 0.088 降到 0.055 m/s。慢时钟、暂停、时间回退测试和三组膨胀区脱困均通过。正式首轮固定障碍 9/12，对向 3/3。右侧失败场景复跑通过。左远场景复跑仍因 IMU 接收超时中断，同夹具的旧版对照通过。[验证汇总](evidence/repair-verification.json) · [正式验收](evidence/qualification-summary.json) · [里程计对照](evidence/odometry-ab.json)

优先同步记录左远场景的 IMU 源时间戳、Gazebo bridge 接收时间、ROS 执行器回调时间和恢复定位状态。用相同夹具交叉复跑两版，确认消息间隔超时发生在哪一层。右侧可对齐停点代价地图、已下发速度和制动距离，针对硬净空拒绝复查。保留接触、净空、IMU 超时和停车检查。

## 复现

在既有 RMUL C072 MID360 + Odin1 的 ROS 2 Humble / Gazebo Fortress 隔离环境中，将本目录以原名放在证据根，容器内路径为 `/evidence/c072-speed200-motion-clock-20261122`。将已验收版本的 GICP 安装前缀链接为 `gicp-install`，执行 `bash build.bash` 构建三个源码包。隔离启动脚本复用交接环境的工作区和 CAD 资源。

在干净的候选副本中执行 `bash validate.bash`，依次运行长路线、脱困、固定障碍和对向测试。正式套件返回非零时，可单独执行 `python3 audit-round.py "$PWD"`、`python3 audit-runtime.py "$PWD"` 检查实跑记录。原始运行用了 `tested-inputs.json` 记录的源码、参数和库哈希。

速度平滑器来自 [Nav2 1.1.20](https://github.com/ros-navigation/navigation2/tree/1.1.20/nav2_velocity_smoother)。上游版权声明和 Apache 2.0 许可证随源码保留。
