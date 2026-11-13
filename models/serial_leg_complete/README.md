# serial_leg_complete：完整 CAD 模型副本

<a id="source"></a>

## 来源

2026-11-13 按用户要求，将 `/home/hzb/wheel_rl_ws/wheelbipe_ros2_sim2sim/src/resources/robot_descriptions/serial_leg_complete` 整个目录复制到当前 package。原始 109 个文件逐字节保留，包括 MJCF、62 个 STL、两份模型说明和 `fusion_script_archive/`。来源目录未修改；本副本使用本目录的相对网格路径，运行不依赖来源工作区。

[copy_manifest.json](copy_manifest.json) 记录来源路径、复制时间、文件大小及 SHA-256。后续在本工作区修改模型后，可据此区分原始模型与算法实验改动。源仓库的 Git ownership 检查阻止读取 HEAD/status，因此未以提交号代替磁盘文件证据。

模型入口为 [robot_passive_limits.xml](robot_passive_limits.xml)。原作者说明见[模型总说明](模型总说明.md)和[限位说明](README_限位说明.md)；这些原文中的历史目录和 Windows 路径按来源原样保留。

<a id="launch"></a>

## 在当前工作区加载

每个终端先执行：

```bash
cd /home/hzb/rm_ws
source ./mujoco_env.sh
```

第一个终端启动隔离 master：

```bash
roscore -p 11321
```

第二个终端启动完整模型：

```bash
roslaunch rm_mujoco serial_leg_complete.launch
```

入口默认暂停，加载原始零位；GUI 中可以查看关节、网格和驱动器。`unpause:=true` 可继续原模型的固定底座、零重力动力学，`headless:=true` 关闭图形界面。服务位于 `/serial_leg_complete/mujoco_server/`。同一 master 一次运行一个仿真入口，避免共享 `/clock`。

此入口直接加载原始 MJCF 的六个 motor，不加载 `RmMujocoRobotHWSim`、BipedalController 或 Manual。后续按用户确认完成的 [ROS 适配模型](../series_legged2/README.md)由 `series_legged.launch` 加载，使用配套 URDF、初态、双 IMU 与弹簧插件。本目录继续保留原始副本，不能仅将 ROS 入口的 `modelfile` 换成本 XML。

<a id="algorithm-use"></a>

## 算法验证的模型基线

本次执行的是完整复制，保留以下源定义：

- 17 个机器人刚体，总质量约 **30.911130235045 kg**；质量、质心和惯量均来自原 XML。
- 16 个铰链：六个轮腿主动关节、八个被动关节、两个无 actuator 的云台关节；四条闭链 connect 约束、两条腿部相对角 tendon 限位。
- 底座 `base` 固定，重力为零，没有地面；步长为 **0.002 s**。
- 六个 motor 的 `gear=1`；原始执行器未配置 `ctrlrange` / `forcerange`。
- 当前坐标下 `right` 位于 +Y、`left` 位于 −Y；髋关节使用 `{side}_hip_thigh` 和 `{side}_hip_output` 命名与源零位。
- `chassis_imu` 位于底座原点，方向为单位四元数，是源模型的占位安装位姿。

这些定义作为本工作区的原始机械输入。自由基座、重力、地面、闭链初态、IMU、关节坐标与控制接口的后续适配位于独立的 [series_legged2](../series_legged2/README.md) 目录。本节下方保留的是原始复制时的静态检查记录；后续接入结果见 [2026-11-13 验证记录](../../docs/verification_20261113.md)。

<a id="verification"></a>

## 本次检查

2026-11-13 核对结果：109 个原始文件的源/副本 SHA-256 全部一致，所有 62 个网格引用均在副本内解析。工作区 MuJoCo 3.3.5 成功加载副本并完成静态渲染；编译模型为 18 个 body（含 world）、16 个关节、6 个 actuator、4 条 equality、2 条 tendon，总质量 30.911130235045 kg。原始零位的 equality 位置残差最大约 7.35e-12 m。

`roslaunch --dump-params rm_mujoco serial_leg_complete.launch` 解析通过，模型与初态路径指向当前 `rm_mujoco`，插件列表为空。检查未启动 ROS 仿真节点或进行物理步进，因此不构成控制器闭环或动作性能验证。
