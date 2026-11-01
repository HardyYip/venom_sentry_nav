# C072：CAD双MID360最小仿真配置

更新：2026-11-01T08:20:47.486021+08:00（Codex）。技术验收通过；发布/保存状态以read.md及publication-receipt.json为准。完整分类及数字见[C072-RESULTS.zh-CN.md](C072-RESULTS.zh-CN.md)。

本轮仅为双MID第一阶段仿真验收。保持既有低速架构、粗略车体、MecanumDrive2及cad-flat-floor；不等同最终实车机构或动力学验收。两MID随大Yaw已确认，现gimbal_yaw仅作粗略代理；完整大小Yaw、全向轮独立悬挂尚未重建。

## 实际硬件与本轮对应

最终机械报告为硬件依据：两台MID360、工业相机、Orin、超电、M3508/行星减速全向轮独立悬挂、大小Yaw、17mm中心供弹500发。两MID均安装在大Yaw上已由用户确认。相机、自瞄/供弹控制不在导航验收范围；Orin、底盘总线与真实制动/受撞/过坎、最终悬挂和精细机构均未仿真验收，不能因CAD位置适配而称已装齐或实车等效。

恰为两GPU lidar，0 RPLidar、0第三台雷达。主20Hz、1875×32并用200Hz IMU；副10Hz、400×32仅作为障碍输入，无原始双雷达点云定位融合，无新增副雷达独立健康停车互锁。两个*_scan子帧只是TF，不是额外传感器。单MID是双MID完整交付后的独立待办，不恢复昨日封存C068。

## CAD坐标与扫描原点

STEP来源source.json引用原微信下载路径，不再复制217603894字节文件；SHA256 c02c13b30b563d35024a92506443c967c37872bd663062b6b874fdd99422eca0。换算X=-CADX、Y=CADY、Z=82.765028mm-CADZ。当前只是壳体几何拟合，尚非光学外参标定。

下表为相对gimbal_yaw的安装原点，单位m/rad，父帧相对base的z为0.2136m：

| 雷达 | xyz | rpy |
| --- | --- | --- |
| 主MID | [-0.02463046921722, 0.11890686455851, 0.21042604288645] | [0, 1.047197551196604, 1.789042845202303] |
| 副MID | [0.02275088350722, -0.11928066106981, 0.21042604288645] | [0, 1.047197551196606, -1.3962634015954634] |

安装原点base z约0.424026m，壳体中心base z约0.430748m，不能混用。GPU扫描相对本体z+0.03m、IMU0保持，header分别front_mid360_scan/aux_mid360_scan，URDF增加真实TF-only子帧。主LIO extrinsic_T=[0,0,.03]、R=identity、在线估计关闭。副转换器/两图使用真实scan帧，严格以消息源戳查TF。

私有PCD还原旧编码外参[.16,0,.256]与Rz90Rx30，共152326点、还原误差约4.77e-7m；扫描原生坐标恒等导入并通过robotbase/gimbal链表达。私有二维边界保守增加一格（1324格），原模型、地图与生产安装不覆盖。两图原footprint_padding均为0.02；0.06是副点云min_obstacle_height，不是padding。

## 入口与运行库

工作区/home/hardy/pb2025_nav_ws，P=src/pb2025_sentry_nav；分支sim/rmul2027-3v3。B=artifacts/rmul_2027_3v3/cad-dual-mid-C072。

| 用途 | 仓库入口/私有前缀 |
| --- | --- |
| 普通CAD导航 | scripts/rmul2027/start_cad_dual_mid360.sh，使用B/install及B/controller-trajectory4-install；普通GICP保持历史普通安装 |
| 显式受控恢复 | scripts/rmul2027/start_cad_recovery.sh，B/cad-metric1-runtime链接C064 safety和B/gicp-metric1-install |
| 独立构建 | build_cad_controller.sh、build_cad_gicp.sh；独立source/build/install前缀，最多2构建线程 |
| 标准检查 | test_cad_dual_mid360.sh、test_cad_recovery.sh、audit_cad_dual_mid360.py；最终具体已验attempt映射见本地accepted-attempts.json |
| 配置/overlay | cad_dual_mid360_profile.py、cad_recovery_profile.py、config/cad_mid360_mounts.json、overlays/ |
| 受保护原版本 | C070 d193ac3061946e20f9cdfd5481ea27cb2e87ca2a；接手HEAD a2aaa651e36e73e979866f4852ebd8d1f4edc2aa |

恢复构建依赖已固定的W/deps/small_gicp fa0cfc983417c044b6036f71dfa448038bb9c820，离线FetchContent，不下载替换。库SHA/算法边界与测试见结果报告。

正式入口必须经过storage_guard，设置RMUL_TEST_ARTIFACT_ROOT为固定任务证据根，不能绕过守护。直接Python夹具须先加载ROS humble、W/install、timing-C067运行环境和configure_dds.sh，并确认import rclpy；motion01遗漏环境的失败证据保留，不重复旧错误。仅一套Nav/Gazebo，无需开RViz。

## 资格与保留边界

本轮最终controller trajectory4＋recovery metric1的14项主资格、额外定向下侧三场及2302项审计全部通过。资格时46公共源完整冻结；交付仅整理纯逻辑测试EOF，6例复验通过，其余45源/31控制器与20GICP构建输入/三库不变。final-source-sha256.json含47份源，另两报告共49文件。4个原dirty子模块、历史3个未跟踪fast XML和C058报告不属于本轮提交。

没有放宽速度、避障圆、到点/停车门槛、质量门槛或恢复预算。当前timing01最后一次故障为ABORTED后预算锁止停车，不能记为取消成功；旧C062最后定位同理。IMU恢复要静止初始化且质量确认后放行。三轮长测是有限仿真证据，不承诺无限运行。

后置任务：单MID独立对照（双MID基线保留）；动态大Yaw同时执行导航长路径单独验证；fast；新定位移植；正式裁判协议；真车/Orin/标定/底盘实际速度与急停；最终轮系/悬挂及精细Yaw几何。当前用户要求只仿真，不启动实车测试。T4相关既有有限故障测试已过，不宣称所有历史死锁唯一归因。

保留C3下侧失败、mapstable1确认失败、旧候选/build与全部raw证据。额外下侧v1虽然到点但没有走下侧，明确为覆盖失败；v2加中间Action确认实际路线，不能将其包装为原下侧失败地图的精确重放。
