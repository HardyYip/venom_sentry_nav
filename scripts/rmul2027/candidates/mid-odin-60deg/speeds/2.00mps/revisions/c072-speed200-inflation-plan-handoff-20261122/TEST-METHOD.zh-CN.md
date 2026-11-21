# 2.00 m/s 提速与膨胀层修复复测

使用本分支既有 C072 MID360 + Odin1 CAD 仿真交接件，ROS 2 Humble、Gazebo Fortress、GPU 无窗口渲染和原测试夹具。隔离运行器沿用交接环境的 `/tmp/rmul-jammy/sim_shell.bash`，将工作区、CAD 资源和证据根挂载到原位置。

1. 将本目录以原目录名置于证据根。容器内路径为 `/evidence/c072-speed200-inflation-plan-handoff-20261122`。在 Humble 下从 `source/rmul_c072_planner` 构建为本目录的 `planner-install`，并将已合格 2.00 m/s 档的 GICP 前缀挂到本目录的 `gicp-install`。用于验证原始模型和加载路径的历史场次以及仿真工作区日志按既有审计脚本保留。
2. 运行 `bash run-standard-round.bash`。顺序为固定 12 场和 0.50 m/s 移动对向 3 场。运行 `python3 audit-round.py "$PWD"`、`python3 audit-runtime.py "$PWD"`。在旁边保留旧 2.00 m/s 档原始逐场目录后，运行 `python3 audit-speed-round.py "$PWD" ../c072-speed200-from-qualified150-20261121`。
3. 分别运行 `bash run-rviz-inflation-test.bash 3.801 -1.942`、`bash run-rviz-inflation-test.bash 3.785 -1.942` 和 `bash run-rviz-inflation-test.bash 3.788 -2.100`。启动点与对应定位初值由夹具设置。发送 RViz 使用的 NavigateToPose Action，注入一次空局部路径，验证末级门先停、向外脱离、到点、拒绝内圈目标及接续新目标。
4. 在同一环境执行 `bash run-speed-benchmark.bash .`，测量 5.2 m 双向直道。将同一脚本指向旧 2.00 m/s 档可复测基线。

`tested-inputs.json` 记录本地验收所用的实际源码、参数及二进制哈希。两份二进制由已验收的安装前缀提供。控制器几何单测见 `inflation-escape-test.cpp`。逐场回执、速度和手动测试记录见[结果](RESULTS.zh-CN.md)。
