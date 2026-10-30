# C071 STEP 几何核对工具

本目录只生成静态装配几何证据，不生成可供导航启动的外参。用户后续已确认两台MID360均随大Yaw转动，详见C071报告最新追加；原几何清单保留当时未确认字段，不是当前安装关系结论。现有C070导航入口不读取本目录清单。

- extract_step_assembly.py：OpenCascade读取累计装配变换、实体包围盒与雷达解析曲面；独立工具环境，不装进ROS环境。
- build_geometry_manifest.py：配对两壳体与两透明罩，记录球心、开口轴、毫米尺寸；不把零件原点或球心当扫描原点。primary/auxiliary只是仿真角色标签。
- layout_contract.py：检查几何元数据；runtime模式始终返回2，改就绪标记也不能放行。仅离线检查，未接入C070启动器，不代表新增全局运行互锁。
- test_geometry.py：19项无ROS/OCP依赖的纯逻辑与负向测试。
- render_geometry_overview.py：需要matplotlib/numpy的包围盒三视图，不是精确遮挡/碰撞验收。

当前目录执行：

    PYTHONDONTWRITEBYTECODE=1 python3 test_geometry.py
    python3 build_geometry_manifest.py /path/to/assembly.json --output /path/to/cad-geometry.json
    python3 layout_contract.py /path/to/cad-geometry.json --mode geometry
    python3 layout_contract.py /path/to/cad-geometry.json --mode runtime
    # 最后一条预期退出码2；不启动ROS。
    python3 render_geometry_overview.py /path/to/assembly.json /path/to/cad-geometry.json --output /path/to/geometry-overview.png

完整STEP读取约305.5秒、峰值RSS约1.95GiB。先复用已保存assembly.json，不自动重读或复制原件。确需重读时用新输出目录，遵守read.md资源守护。实际使用独立target的cadquery-ocp-novtk==7.8.1.1.post1，CPython3.10 Linux wheel；只对CAD命令设置PYTHONPATH，不改变系统或导航依赖。

运动父节点已确认为大Yaw；后续需完成仿真CAD/base映射及扫描/IMU原点约定，再实现独立可选CAD profile。双MID全部验收后新增独立单MID对照任务，旧C068仍归档。当前未修改挂载、重力、碰撞体或Nav2，新挂载全部导航回归未运行。详见../reports/C071-RESULTS.zh-CN.md。
