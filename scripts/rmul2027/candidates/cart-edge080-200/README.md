# 2.0 m/s 玩具小车贴边仿真

本候选保存 2026 年 11 月 27 日 13:29 完成的 Gazebo 小车仿真。控制器请求经过贴边限速，再由平滑器输出给虚拟底盘。平移速度上限为 2.0 m/s。

| 测量项 | 本轮结果 |
|---|---:|
| 到点并停稳 | 15/15 |
| 接触次数 | 0 |
| 贴边有效时间 | 8.82 s |
| 贴边均速 | 0.5701 m/s |
| 最小箱体净距 | 12.27 cm |

距离按车中心至最近障碍表面计算。0.85–0.95 m 距离带内，里程计速度至少为 0.03 m/s 的样本计入时间加权均速。0.80 m/s 的贴边指标本轮未通过。[逐场评分](edge-score.json) · [导航审计](native-nav/full-rerun-1795757044818/audit.json)

## Ubuntu 桌面手动选点

准备下述 Gazebo 缓存后，在本目录运行 `bash manual/open.bash`。入口读取当前 X11 的 DISPLAY 和 XAUTHORITY，启动双固定障碍场景。黄色车身表示小车，点击 **Nav2 Goal** 后在浅灰区域拖出目标朝向。关闭 RViz2 会结束本次仿真。桌面窗口实测见[车身截图](manual/cart-window.png)。

若 Humble 的 RViz2 在独立前缀，将环境变量 CART_RVIZ_PREFIX 指向含 lib/rviz2/rviz2 的目录。

## 复核和运行

在本目录运行 `python3 score.py` 可用已附地图和 15 场记录重算评分，需要 NumPy、PyYAML 和 Pillow。程序将评分写到标准输出，贴边速度未达标时返回 1。

Gazebo 复跑沿用原验收机的 Ubuntu Jammy、ROS 2 Humble、Gazebo Fortress 缓存与 bubblewrap。将原 runtime 目录链接到本目录的 runtime，将完整 c072-verify-headon-planar-accel-20261123 归档链接到本目录的同级，将原 safety-install 的实际文件复制到 native-nav/safety-install。运行库指纹见 [preflight.json](native-nav/preflight.json)。Git 仓库包含测试程序和逐场数据，运行时缓存和已编译插件需单独提供。

准备好依赖后，在本目录运行以下命令。每次复跑使用新的轮次名称。

```bash
cd native-nav
round="rerun-$(date +%Y%m%d-%H%M%S)"
./isolated.bash bash /trial/contact-check.bash "$round-contact"
python3 run-round.py "$round"
python3 audit.py "$round" "$round-contact"
cd ..
python3 score.py --round "$round"
```
