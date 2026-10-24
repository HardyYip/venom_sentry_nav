# C052 普通导航候选验收与采用

Codex，2026-10-24T09:25:37.849145+08:00。本轮规定的有限矩阵全部通过，普通0.5m/s入口采用本候选。fast配置不变，本轮没有重新验收fast。

| 场景 | 结果 | 关键数值 |
| --- | --- | --- |
| 固定四类各3次 | 单批12/12，全新启动，准备中止0 | 最小名义净距0.114678m，最大真值终点误差0.149616m |
| 单个同速对向物体 | 全新3/3，双方额定0.5m/s | 最小名义净距0.092744m，最大真值误差0.154691m |
| 原危险接近末段重建 | 3/3 | 净距0.162099/0.065956/0.087041m；不是完整人工往返 |
| 普通完整路线 | 8项通过 | 静止就绪、6个到点、1个占用目标正确拒绝；最大真值误差0.181306m，最长46.702s |
| 停放障碍移除 | 通过 | 绕行到点，移除后global/local两图采样cost均0 |
| 封路后清除恢复 | 通过 | 原生6且无运动，安全停车；移除后原生4到点 |
| 控制器独立还原 | 16个改动/新增文件一致 | 固定基准应用完整patch，隔离ROS-only构建、CMake测试及实际pluginlib载入通过 |

固定/对向/末段及两类清除场景的真实导航接触为0，正对照均有原始接触消息。固定12采用right-control、left-near、left-far、left-reverse顺序，每组3次；只调整执行顺序，原run_case几何、保护与阈值不变。未拼接C041/C050/C051通过，也未在本候选失败后反复重试到通过。

原1秒停车首快照的实际/指令平移及角速度四项均小于0.03，原2秒观察预算、0.2秒连续稳定确认保持。运动目标原生SUCCEEDED4、180秒、估计/真值终点误差均不超过0.35m保持；占用目标与完全封路的status6为预期拒绝，不能算到点。

移动代理基于仿真时间驱动；三次实际轨迹回归速度0.500018/0.499630/0.500229m/s。我方额定0.5m/s，真值采样峰值0.555339/0.563259/0.522765m/s，减速绕行并不要求双方瞬时等速。session末帧的opponent_measured_speed_mps=0是物体结束后静止快照，不能替代运动区间回归速度。真值仅驱动代理和评估，不进入我方规划。

## 修改与判断

此前C050固定9过后，right-control-01准备导航失败；C051右侧第一例通过、第二例准备失败，两次失败均保留且未计为碰撞或停车失败。C052只在C051可达路线距离中增加软余量代价：硬边界外两格内，正边权从1平滑增至2，以Dijkstra选更留空间的路线。同一硬安全图、完整自旋圆半径0.432883794m、未知/图外/首点拒绝、原DWB障碍critic、原异常回调、原GoalDist缩放、额定速度及参数均保留。

14项针对性单元通过，含30000格与实际Spin圆核对照；具体贴墙偏好回归在旧C051头文件上独立复现失败（77对79），在C052通过。隔离恢复构建也通过CMake接入的测试目标。历史实验Omni两项可选功能仍默认关闭；普通采用CollisionAwareOmniController。

已核对本机DWB1.1.20二进制及原始源码：原GoalDist波前忽略障碍。此前“253阻断原GoalDist波前”的推断错误，已在read.md及原推断文件纠正。该启发式本身不被称为上游缺陷，有限通过也不证明旧问题唯一根因。

## 版本与复现

- 参数SHA256：3dd610e809375a29f5467790fb00bcccdfb03666d569b4c0fa312088e7df3a10
- 控制器入口CPP：2e308daa6a8ff9396c8b80d94409c7f9fdd5924d7e8db009c232aa5cf23fd69d
- 新geodesic头：db57c76d7b395b4c708f57786e2b571185ef99a1115a464f6ec9a0241788d9cc
- 实测运行库：bf6b4c496aa4713f4615e38d41ded64bb6fbc13c035678aa8cb02fc3abdbb611
- 控制器基准：b542c5794b91fba2bb3fe2cbdc72fcf854fedb1a；完整patch SHA256：33f069c088be30d86c9911c0cd30c06c04757cd51636da9dad4767122cfcbdd5
- 干净构建库：4c1847dfe695c17c7b6f35a15dc43aa743eb0b90a2073965e34b9f27d18c6dbf。构建路径不同，未声称与实测库逐字节相同。
- 每次真实startup/end参数、CPP、库、actor、BT、新header、spin和package身份均匹配，目标UUID时段被快照包围，21/21严格来源审计通过。
- 控制器26个开测前文件保持至测试结束；测试后只更新README说明，编译源/配置不变。规范patch恢复当前16个改动文件一致。
- worker启动时已载入旧审计器，末尾因写死废弃BT文件名而审计失败；原失败状态和审计保留。新进程用正确原BT精确路径并额外核查头文件等，21/21通过。无导航重试。
- Ubuntu22.04、ROS Humble、Gazebo Fortress6.18；DWB/Navfn1.1.20。确切Debian版本见tested-navigation-packages.tsv；dwb_core、dwb_plugins、dwb_critics及ament_cmake_gtest依赖已声明。

队友按README的prepare_rmul2027.py、rosdep及build_rmul2027.sh恢复。manifest新增控制器patch，gitlink仍锁定可获得的b542c57基准；已有四份依赖patch不变。旧普通参数保存在history/before-C052-nav2_test.yaml。普通入口start_rmul2027_test.sh选择与实测逐字节相同的新参数。用户需手动测试时才启动单个RViz。

## 证据与边界

完整原始证据：/mnt/e/RMULArtifacts/pb2025_nav_ws/codex-clearance-metric-C052-20261024-0826
关键文件：full-manifest.json、full-raw-audit.json、full-runtime-provenance.json、codex-code-review.json、source-review/source-build-review.json、unit.log、independent-reproduction-result.json、native-clock-window-analysis.json。仓库C052-summary.json保存紧凑结果；原始日志不纳入Git。

本轮原生clock覆盖48个owned-goal窗口，缺失0，连续采样最长同戳跨度0.134126s；另有24个observer调度空档，最长约1.354s，不能据空档声称观察到Gazebo停顿。历史T4时序故障根因仍未知，不能写已修复。

固定/移动的净距为几何代理和采样计算，不是连续精确物理间距证明。零戳代价图按接收时间使用，未证明源时间戳TF全帧覆盖；完整人工往返未运行，新增fast回归、横穿、多车、实车和战术均未运行。不能把本轮有限路线通过写成全场可靠。

本轮仅一套后台Gazebo，无RViz、无整段bag，原容量50/40/30GiB及512MiB守护保持。旧C027/C041/C049/C050/C051及其他原始失败均保留。构建/测试完成后清理本轮可重建临时文件并保存清单。
