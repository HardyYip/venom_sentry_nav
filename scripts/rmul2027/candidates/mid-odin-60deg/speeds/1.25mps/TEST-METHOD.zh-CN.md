# 1.25 m/s 速度相关制动时间的局部评分

沿用前一候选的定位交接补丁、车体、传感器、控制频率和仿真验收。唯一区别在局部扫掠净空评分。原实现只检查前 0.65 秒，用的是 0.50 m/s 的制动时间。新实现读取纵向与横向减速度，按测得速度和采样速度中的较大值计算制动时间，再计入原 0.15 秒延迟。评分最短观察 0.65 秒，最长到现有轨迹末端。每条轨迹仍接受原碰撞与自旋净空检查。

前版定向三个左近场通过、正式左近与右侧对照仍有失败。本版直接用同一配置重新验收四组固定线路各三场、移动对向三场。用 run-standard-round.bash 执行完整轮，随后用 audit-round.py 检查原始回执。定向回归独立统计。

复跑使用既有 RMUL CAD 仿真工作区和已记录的 Humble 依赖。将此候选目录挂载为 /evidence/c072-speed125-clearance-horizon-20261121。在隔离环境中先加载 /opt/ros/humble/setup.bash 和工作区 install/setup.bash，再由候选根目录执行下列构建。参数中引用的旧版行为树由本目录 source/scripts/rmul2027/behavior_trees/navigate_field_refresh.xml 提供，按参数所记路径放置同一文件。

```bash
colcon build --base-paths source/scripts/rmul2027/packages/rmul_c072_planner --build-base planner-build --install-base planner-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
colcon build --base-paths source/c072-localization/small_gicp_relocalization --build-base gicp-build --install-base gicp-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DFETCHCONTENT_SOURCE_DIR_SMALL_GICP="$RMUL_WORKSPACE/c072-transfer-source/small_gicp_dependency"
```

使用随附仿真脚本覆盖既有 CAD 工作区对应入口。宿主通过已有 sim_shell.bash 挂载运行环境，同一轮运行 run-standard-round.bash 后执行 python3 audit-round.py 本候选绝对路径核对真值与配置回执。原始工作区和 CAD 仿真资源仍为依赖。
