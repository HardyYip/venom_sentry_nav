# 磁盘保护与录包规则

2026-10-21事故：无上限原始点云录包使D盘宿主空间耗尽。当前修正不改变导航参数。

## 默认与门限

- 所有七个仿真/路线/故障/避障/旧图验收Python入口自动进入同一守护，嵌套入口共用本轮预算。
- Windows宿主盘和实际输出分区：启动至少50GiB，40GiB预警，30GiB停止；读盘失败也停止。
- 普通测试默认不启动rosbag。需要轻量话题时显式加 --record-bag，不录原始/注册/地形点云。
- 单轮新增产物512MiB、产物根目录累计4GiB、单轮宿主实际分配增长2GiB。任一到线即停止，不自动删证据腾空间继续。
- 轮询每秒一次，关停有短暂缓冲，所以这些是停止预算而非硬配额。普通测试最长30分钟；单独手动仿真最长2小时。
- 守护使用本轮唯一环境标记处理脱离进程组的子进程，匹配PID身份后发停止信号，不清理其他用户进程。
- 状态和通知默认放独立宿主盘；本机固定E盘，不清理C盘。桌面通知受Windows设置影响，状态JSON和控制台同时保留。
- 不安装全机开机监控；手动绕过工程入口运行其他命令不在守护范围内。

## 本机目录与重现

工作区 .rmul-storage.json 不在主仓库中，保存本机选择：

    {
      "host_root": "/mnt/d",
      "artifact_root": "/mnt/e/RMULArtifacts/pb2025_nav_ws",
      "state_dir": "/mnt/e/RMULStorageState/pb2025_nav_ws",
      "windows_python": "/mnt/d/Anaconda/python.exe"
    }

E盘必须挂载，缺失即拒绝运行，不悄悄改写C盘或VHD。环境变量 RMUL_STORAGE_HOST_ROOT、RMUL_TEST_ARTIFACT_ROOT、RMUL_STORAGE_STATE_DIR 可明确覆盖位置。其他原生Linux电脑默认检查其工作区分区；其他WSL电脑应明确指定VHD所在的Windows宿主分区。

自动测试的 --record-dir 必须在 artifact_root 下且每次用新子目录；旧的WSL内历史命令会明确拒绝。原地图、PCD、CAD及依赖资源位置保持原样。小型仿真日志仍可留工作区logs，但也计入本轮512MiB增量预算和宿主2GiB增长预算。

例如在工程目录运行原入口不改变命令；要录轻量bag时给相应Python测试脚本加 --record-bag，输出选 /mnt/e/RMULArtifacts/pb2025_nav_ws/<新的本轮名称>。脚本控制台会记录守护的产物和状态目录。

完整路线的 --record-pipeline 已经禁用。确需检查原始输入，应在已有仿真上单独运行：

    python3 scripts/rmul2027/record_raw_window.py --record-dir /mnt/e/RMULArtifacts/pb2025_nav_ws/raw-<新名称>

该入口只录一个点云源、IMU、TF和时钟，128MiB或10秒先到即停，停止预算退出码74，不能据此误报导航失败。分卷32MiB、内存缓存64MiB；分卷不是总容量上限。

## 验证与已知边界

已完成11项逻辑检查、真实Linux时长触发以及脱离进程组且忽略INT/TERM的子进程清理。测试只用睡眠进程，未启动Gazebo或新导航录包。桌面原生API已接受通知请求，不等于用户一定看见。

测试文件为 tests/test_storage_guard.py。在WSL运行时让 RMUL_STORAGE_TEST_SCRATCH 指向E盘已有测试临时目录，测试不会创建大文件。

本轮未重新运行导航完整路线；C024候选的既有验收状态不因存储修正变成通过。历史保留包是事故前证据，不按新4GiB预算强行删除；新产物目录独立计量。当前C024和未完成/失败包仍保留，后续按证据价值有选择地归档，不再无上限积累。

清理原始包意味着失去对应回放能力。旧报告与配置保留，原始包已删除的范围应以恢复记录中的清单为准。2026-10-21确认删除的33个旧包及微信cache记录在E盘 CodexDiskRecovery/C024，清理后已离线compact并测量Windows实际余量。
