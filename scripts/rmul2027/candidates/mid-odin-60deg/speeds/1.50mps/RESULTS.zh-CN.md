# 1.50 m/s 同版仿真

正式固定 12/12、移动对向 3/3。15 场均为零接触、真值到点且完成原一秒停车。
固定最小净距 0.11525 m。

| 对向场次 | 本车实测峰速 (m/s) | 最小物理净距 (m) |
| --- | ---: | ---: |
| headon-01 | 0.92725 | 0.33184 |
| headon-02 | 1.05589 | 0.19591 |
| headon-03 | 1.08944 | 0.32524 |

1.50 m/s 是配置上限。表中速度由仿真真值计算。

## 同档历史复测

| 候选 | 通过 | 失败编号 |
| --- | ---: | --- |
| c072-speed150-footprint-cost-20261121 | 14/15 | headon-01 |
| c072-speed150-from-clearance-horizon-20261121 | 13/15 | headon-01, headon-03 |
| c072-speed150-registration-continuity-20261121 | 13/15 | left-far-02, left-near-02 |
| c072-speed150-replan-after-stop-20261121 | 14/15 | headon-02 |
| c072-speed150-from-verified-clearance-20261121 | 14/15 | left-far-03 |
| c072-speed150-preserve-clearance-cost-20261121 | 12/15 | left-near-02, left-reverse-01, left-reverse-02 |
| c072-speed150-continuous-clearance-20261121 | 11/15 | left-far-03, left-near-02, left-near-03, left-reverse-03 |
| c072-speed150-historical-planner-20261121 | 14/15 | headon-01 |
| c072-speed150-forecast-clearance-20261121 | 14/15 | headon-03 |
| c072-speed150-verified-recovery-seed-20261121 | 14/15 | left-near-01 |
| 本轮 | 15/15 | 无 |
