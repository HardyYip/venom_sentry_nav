# 60° MID360 ＋ Odin1 低速仿真验收

固定 12/12、对向 3/3。固定最小净距 0.153 m、对向最小净距 0.246 m。零接触，所有正式目标到点并在原一秒窗口停稳。MID360 保持原位和 60° 仰角，沿用 Odin1 与原车体。此验收发生在已准备的 ROS 2 Humble／Gazebo Fortress 仿真中。

该目录以已验收的 C072 双 MID 代码为参考，保存本次 MID/Odin 仿真源码覆盖层、参数、验收方法和实测摘录。原 2026-11-19 的 MID/Odin CAD 交接件提供仓库基线尚未纳入的安装和模型。将 `source/scripts/rmul2027` 与 `source/rmul2027_decision` 合入该交接工作区，在隔离前缀编译 `rmul_c072_planner` 和 `source/c072-localization/small_gicp_relocalization`；依据检测方法使用同版输入运行固定和对向夹具。两组正式记录均执行同一个参数摘要。

[检测方法](TEST-METHOD.zh-CN.md) · [本次改动和结果](RESULTS.zh-CN.md) · [逐场证据](qualification-summary.json)
