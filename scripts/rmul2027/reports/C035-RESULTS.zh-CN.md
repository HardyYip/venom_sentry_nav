# C035：全关自旋候选未采用

候选只把 C034 的 init_spin_speed 从 3.14 改为 0.0，参数 SHA256 为 `bc60d4cdec6ae8e04552ba7f5ef83eef1db9e9fd9f1dcac73314374a6cc8bfcc`。中心点前缀 4/4 通过原 1 秒全零停车，固定近/远两组 6/6 通过。

反向首例 left-reverse-01 名义最小足迹净距 0.049304270m，触发 dangerous_approach，真实接触 0，按保护停止，判失败。terrain ROI 无合格点，2D 回波和双图标记较晚出现；不能因此认定 3D 原始盲区已修复，也不能简单全关自旋。

固定剩余 5 次、移动、完整路线、停放移除及封路恢复均未运行。候选不采用；普通默认当时仍为 C034。该失败不被后续候选成功覆盖。

证据：E:\RMULArtifacts\pb2025_nav_ws\completion-C035，尤其 fixed/left-reverse-01/session.json、first-fault.json 和 statistics.jsonl（若归档压缩，见同轮证据压缩清单）。C034 的原 1 秒中心点停车失败也继续保留。
