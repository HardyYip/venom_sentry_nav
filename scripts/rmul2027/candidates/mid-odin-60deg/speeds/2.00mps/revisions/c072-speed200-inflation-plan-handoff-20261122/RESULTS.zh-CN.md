# RMUL 2.00 m/s 提速与膨胀层修复

同一份候选实测 **固定 12/12、对向 3/3**，15 场零接触、到点并通过一秒停稳。固定最小物理净距 **0.1273 m**，移动对向最小 **0.1895 m**。原两轮未过的左远 01 和对向 03 均通过。[完整回执](qualification-summary.json) · [运行库核对](runtime-libraries.json)

## 同路线测速

Gazebo 5.2 m 直道往返，速度取仿真真值峰值，用时取导航 Action 从发目标到返回成功的仿真时间。两版均最终停稳。

| 路段 | 真值峰速 m/s，原版 → 修复版 | 用时 s，原版 → 修复版 |
| --- | ---: | ---: |
| 去程 | 1.066 → 1.261 | 11.77 → 10.59 |
| 回程 | 1.011 → 1.368 | 9.82 → 8.11 |

正式固定绕障段的逐场峰速中位数 0.774 → 0.827 m/s；移动对向为 1.106 → 1.121 m/s。平移上限为 2.00 m/s。[同名场次逐项速度对照](standard-round-speed-comparison.json) · [直道结果](speed-benchmark-0miBRV/results.json)

## 手动目标及膨胀层

三次从安全停住的位置发送 RViz 同一个 NavigateToPose Action，起点代价依次为 209、253、253。三次注入一份空局部路径后，末级门均先停，接收重新验证的脱离路径及新指令后，在约 0.30 秒恢复输出。脱离指令峰值 0.20 m/s，三次脱至代价 171，随后到达原目标。三次试发深层膨胀内目标均拒绝且无移动，随后的可达目标均到点停稳。验证行程中实体车体包络与观测到的原始占用格、未知格之间的距离。[三次原始回执及安全门时序](rviz-inflation-summary.json)

控制器按前方可用制动距离限制朝内圈的速度。已在内圈时先确认停稳，再用扫掠包络验证朝外的路段，并让同戳的经验证局部轨迹经过原末级安全门。普通导航使用 8 × 8 m 局部地图、3.25 m 前向路径、每轴 ±1.75 m/s² 规划加减速和 8 cm 到点门槛。[参数](params.yaml) · [推进与脱离控制器](source/rmul_c072_planner/src/inflation_aware_controller.cpp) · [出口几何](source/rmul_c072_planner/src/inflation_escape.hpp)

验收范围为本机 ROS 2 Humble / Gazebo；手动目标走实际 RViz 使用的 Action 接口。[汇总](final-verification.json)
