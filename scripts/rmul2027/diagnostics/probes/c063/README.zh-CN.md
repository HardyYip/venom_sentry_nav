# C063 已安装Nav2库的独立探针

仅诊断程序，不安装导航插件、不链接决策BT4、不改生产安装。通过真实已安装ObservationBuffer和StandardTrajectoryGenerator验证高度过滤与减速积分，保留原配置作为负对照。仿真真值不参与本探针。

先source ROS Humble，再在同一任务容量守护内运行：

```bash
cmake -S scripts/rmul2027/diagnostics/probes/c063 -B "$RMUL_ARTIFACT_DIR/nav2-probes"
cmake --build "$RMUL_ARTIFACT_DIR/nav2-probes" -j1
ROS_DOMAIN_ID=77 ctest --test-dir "$RMUL_ARTIFACT_DIR/nav2-probes" --output-on-failure
```

ObservationBuffer检查变换后0.03/0.059m过滤、0.061/0.15/0.25m保留（map和odom）；轨迹探针对比零/低速/匀速目标，断言固定时间积分保留制动位移、首输出0.45m/s对应20Hz下1m/s²减速。不是完整避障或实车验收。

第三个footprint探针用真实Nav2 padFootprint/calculateMinAndMaxDistances验证：global的内切规划半径必须覆盖local外接半径加半格对角。原padding0.02不满足，0.07满足；仅对应本轮32边轮廓及0.05m分辨率，不外推其他机器人。

采用状态更正：扩大local圆形与global padding0.07/提高障碍权重的候选虽通过几何探针，但被实体绕行回归拒绝，未采用。第三套几何探针仅保存该诊断的正负对照，不能替代联调资格。最终minimal候选恢复原C052矩形local/原global及原权重，仅采用前两套直接证据支持的高度与时间积分修正。
