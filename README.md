# rm_mujoco：当前工作空间的 MuJoCo ROS 入口

<a id="overview"></a>

## 配置与职责

2026-11-09 按用户要求接入当前 catkin 工作空间。采用本地 `mujoco_ros_pkgs` 的 `noetic-devel`（`aadba6308975ec14bb4f008c1b86f8177dc4f71a`）、MuJoCo **3.3.5**、GLFW **3.3.10**。两个 ROS 包来自 `/home/hzb/Documents/`；原目录没有修改。引擎使用官方 3.3.5 Linux x86_64 发布包，安装在工作区 `.local/mujoco-3.3.5`，GLFW 在同一前缀内构建安装。未改系统库、bashrc 或全局环境。

运行链为：

```text
mujoco_ros / mujoco_server
  → mujoco_ros_control / MujocoRosControlPlugin
  → rm_mujoco / RmMujocoRobotHWSim
  → 官方 controller_manager
  → 当前 rm_chassis_controllers / BipedalController
```

生产控制器直接使用当前 `src/rm_controllers`，没有复制算法。控制器参数唯一来源为 [series_legged2.yaml](../rm_config/config/rm_controllers/series_legged2.yaml)。2026-11-13 活动模型改为 [完整 CAD 适配模型](models/series_legged2/README.md)：连杆几何取 `series_legged`，IMU、云台、transmission 与腿部限制取 `series_legged2`，质量和惯量取完整 CAD；另加载用户拟合曲线对应的被动弹簧。底盘与云台控制器的原参数均保持现值。[旧模型](models/series_legged/README.md)保留供历史记录对照。

本入口不依赖旧 `mujoco_platform` 的运行器、Python 环境、桥接库或 GroundCore。确认无活动依赖后，已按用户授权直接删除旧目录（含其中的实验和构建产物）及根目录旧 `mujoco` 脚本，不留备份。当前模型及其生成输入独立位于本 package。

<a id="complete-model"></a>

## 完整 CAD 模型副本

2026-11-13 按用户要求，将 `wheel_rl_ws` 中的 `serial_leg_complete` 完整复制到 [models/serial_leg_complete](models/serial_leg_complete/README.md)，包含原始 MJCF、62 个网格和来源资料。`roslaunch rm_mujoco serial_leg_complete.launch` 保留原始固定底座、零重力预览，默认暂停。用于当前底盘和 yaw/pitch 控制的派生模型位于 [models/series_legged2](models/series_legged2/README.md)，由 `series_legged.launch` 加载；原始副本不被改写。

<a id="quickstart"></a>

## 启动

每个终端先执行：

```bash
cd /home/hzb/rm_ws
source ./mujoco_env.sh
```

脚本加载当前工作区和引擎库，设置 `ROBOT_TYPE=series_legged2`、`ROS_MASTER_URI=http://127.0.0.1:11321`、`ROS_IP=127.0.0.1`，清除 `ROS_HOSTNAME`。不使用原终端中指向实机的 master。需要另一个本地端口时，先设置 `RM_MUJOCO_PORT`，所有相关终端使用同一值。

第一个终端启动隔离 master：

```bash
roscore -p 11321
```

第二个终端启动图形界面：

```bash
roslaunch rm_mujoco series_legged.launch
```

无图形界面：

```bash
roslaunch rm_mujoco series_legged.launch headless:=true
```

当前 `manual` 默认值为 `true`，启动文件已包含一个 `rm_manual`。Manual 加载 joint_state、robot_state、chassis 和 gimbal 四个控制器，先运行状态控制器，收到有效 DBUS 输入后启动底盘和云台。不要再运行 `roslaunch rm_config manual.launch` 或 `mon launch rm_config manual.launch`；它们会替换同名 Manual 节点，并覆盖仿真所需的控制器清单。机器人初始为无运动命令状态；初始摆放不等于控制器已经完成起立。GUI 中可暂停/继续；也可以通过上游 `/mujoco_server/set_pause` 服务控制仿真。`unpause:=false` 会让控制器启动切换等待仿真步进，恢复仿真后完成。

可用参数：`robot_type`（参数文件选择，默认 `series_legged2`）、`modelfile`、`initial_joint_states`、`robot_description_file`、`imu_config`、`mujoco_plugin_config`、`headless`、`unpause`、`realtime`（默认 1.0）、`controllers`、`manual`（默认 `true`）。模型、ROS 描述、初态、IMU 和插件配置需成套对应。双 IMU 使用 [config/imus.yaml](config/imus.yaml)。物理步长与控制目标周期均为 0.001 s；不将它们等同于墙钟间隔。

仅查看模型、不加载控制器时使用 `manual:=false controllers:=false`。插件仍提供 controller_manager 服务。

<a id="commands"></a>

## 命令与 Manual

直接发布命令测试时，先用 `roslaunch rm_mujoco series_legged.launch manual:=false` 启动；spawner 会启动上述四个控制器。使用原 `/cmd_vel`、`/cmd_chassis`、`/leg_cmd`，三个终端持续发布的示例：

```bash
rostopic pub -r 50 /cmd_vel geometry_msgs/Twist '{}'
rostopic pub -r 50 /leg_cmd rm_msgs/LegCmd '{jump: false, leg_length: 0.20}'
rostopic pub -r 50 /cmd_chassis rm_msgs/ChassisCmd '{mode: 0, command_source_frame: base_link, follow_source_frame: yaw, accel: {linear: {x: 5.0, y: 5.0}, angular: {z: 10.0}}, power_limit: 100.0}'
```

三条命令需要分别持续运行；`cmd_vel` 的到达时间驱动原 ChassisBase 超时逻辑。`mode: 0` 是 RAW 请求，机械模式仍由当前控制器决定。需要观察运动时修改 Twist 的 `linear.x`/`angular.z`；初始低幅值验证后再按具体任务测试。发送 `mode: 4` 是原 FALLEN 清零请求。停止仿真用 roslaunch 所在终端的 Ctrl-C。

使用 rqt 虚拟遥控器时，使用包含原 Manual 的启动方式（当前默认）：

```bash
roslaunch rm_mujoco series_legged.launch manual:=true
```

[主启动文件](launch/series_legged.launch) 统一加载控制器 YAML 一次，再包含 [Manual 启动文件](launch/series_legged_manual.launch)。子文件加载原 Manual YAML，直接使用其中的状态控制器列表，只调整主控制器为底盘和云台、校准控制器为空列表。子文件依赖主入口已加载的控制器参数。原 Manual YAML 的 `gimbal_calibration` 已直接写为 `[]`。不加载 shooter；底盘自身提供所需姿态 TF，不额外加载 orientation 控制器。Manual 自己管理加载及启停，因此此时不启动额外 spawner。原 DBUS topic 仍是 `/rm_ecat_hw/dbus`，不会凭空产生遥控器输入；此处只连接隔离 master，不能直接接入实机主站。不要同时用 Manual 和独立发布器争用运动命令。

`debug_gimbal_flag` 唯一来源是 [原 Manual YAML](../rm_config/config/rm_manual/series_legged2.yaml)，当前为 `false`。MuJoCo launch 不再提供同名参数或覆盖它。需要仅调云台时，在原 YAML 中设为 `true`；此时 `SeriesLeggedManual::updateRc()` / `updatePc()` 会持续发出 `FALLEN`。该参数在 Manual 构造时读取，改动后需要重启 Manual；仅执行 `rosparam set` 不会更新已运行的对象。

另一个终端同样加载 `mujoco_env.sh`，再执行 `rqt`。Virtual Joystick 的 Topic 使用 `/rm_ecat_hw/dbus`，开启底部发送开关。右拨杆中位（`s_r=3`）为 RC 摇杆模式，上位（`s_r=1`）为 PC 键鼠模式；键盘输入还需启用插件的键盘捕获并给其焦点。

遇到“控制器 running 但机器人不动”时，可以只读检查：

```bash
rostopic hz /rm_ecat_hw/dbus
rostopic echo -n 1 /cmd_chassis
rostopic echo -n 1 /controllers/chassis_controller/legged_chassis_mode
rosparam get /rm_manual/debug_gimbal_flag
rosparam get /rm_manual/controllers_list
```

DBUS 持续更新、Manual 输出 `mode: 4` 且控制器显示 `SIT_DOWN` 时，应先检查上述调试参数及操作模式。rqt 的 controller manager 能列出 `running` 只表示控制器已运行，不表示底盘收到起立/运动命令。此次现场诊断与修复验证见 [rqt 通信记录](docs/rqt_connection_20261113.md)。

2026-11-09 的旧模型记录确认 Manual 可初始化。原 Manual YAML 缺少部分功率阈值、`follow_source_frame`、`chassis_motor` 等参数，原框架会打印缺项告警；这里没有猜测数值或借用其他车型补齐。新模型的验证范围见 [2026-11-13 记录](docs/verification_20261113.md)，完整遥控器/键鼠链未作动作验收。

云台普通 RATE 命令入口是 `/controllers/gimbal_controller/command`，消息为 `rm_msgs/GimbalCmd`，`mode: 0`，`rate_yaw` / `rate_pitch` 单位 rad/s；命令的 `stamp` 使用当前 ROS 仿真时钟。

<a id="build"></a>

## 构建与检查

```bash
cd /home/hzb/rm_ws
source ./mujoco_env.sh
catkin build mujoco_ros_control rm_mujoco rm_chassis_controllers rm_gimbal_controllers robot_state_controller rm_manual --no-status -j2 -p1
catkin build rm_mujoco --no-deps --no-status -j2 -p1 --make-args hardware_interface_test complete_model_test
rostest rm_mujoco hardware_interface.test
rostest rm_mujoco complete_model.test
```

若要重编译整个工作区，包括清理 `build/`、`devel/` 后的首次构建，仍须先加载项目内的 MuJoCo/GLFW 环境：

```bash
cd /home/hzb/rm_ws
source ./mujoco_env.sh
catkin build --no-status -j2 -p1
```

2026-11-10 修复了全工作区重编译时发现的消息清单问题：`rm_msgs/CMakeLists.txt` 曾声明不存在、且当前源码没有使用的 `PowerModelFit.msg`，导致消息生成配置失败，后续包被标记为 `Abandoned`。现已删除该无效声明；不需要补造消息定义，也不需要恢复旧 `mujoco_platform`。

同日使用 `catkin build --continue-on-failure --no-status -j2 -p1` 检查全部包，结果为 **45/46 成功**，没有被中止的包。唯一失败项是 `rm_referee`：其解包代码使用的三个 EventData 字段和两个功率字段与当前消息定义不一致。按用户决定，本轮不修改裁判系统，因此全工作区构建仍会在该包失败。上方列出的 MuJoCo、底盘控制器、RobotStateController 和 Manual 相关包均已编译通过，可继续使用相关包的构建命令。构建日志保存在工作区 `logs/mujoco_integration/20261110/`。本轮只验证编译，没有启动仿真或实机节点。

上述硬件测试使用独立 rostest master。旧模型测试保留；完整模型测试验证八路 effort、八个被动状态、双 IMU、闭链初态及弹簧势能梯度/内部力一致性。它们不评价控制器运动性能。新模型结果见 [验证记录](docs/verification_20261113.md)，旧结果见 [2026-11-09 记录](docs/verification_20261109.md)。

复现短集成检查时，在一个新仿真会话使用 `manual:=false unpause:=false`，等待控制器加载，再从另一个已加载环境的终端执行：

```bash
python3 src/rm_mujoco/tests/ros_smoke.py logs/mujoco_integration/ros-smoke.json --quick
```

脚本通过上游 step 动作发送 RAW 零速、短平移、yaw/pitch 速率命令和 FALLEN，并执行底盘 stop/start；命令、Mode、兼容状态、JointState、模型服务状态分别保存。`--quick` 的命令阶段约 0.4 s，省略时延长检查阶段。该脚本会产生仿真运动，只用于新建的独立 localhost 会话；没有姿态扶正或内部状态改写，也不判定站稳或云台跟踪精度。

在新建的隔离 master 中，上游初始示例也可单独检查，避免复用之前的轮腿插件参数：

```bash
roslaunch mujoco_ros launch_server.launch use_sim_time:=true
```

轮腿仿真只需前述相关包；全工作区构建也会编译 laser、mocap、sensors 等可选插件，但不会自动在仿真中加载它们。

<a id="interfaces"></a>

## 接口与时间语义

- `MujocoPlugins` YAML 使用上游结构；硬件类型和 `control_period` 位于 `hardware` 层。活动配置使用上游默认的根命名空间和 `robot_description` 参数名。
- 八路 effort 继承本地 Noetic 版 `DefaultRobotHWSim::writeSim()`，经指定 URDF 限值写入 `qfrc_applied`。模型没有另一路 motor actuator；弹簧通过独立的被动力插件写入 `qfrc_passive`。
- 被动关节仅注册 JointState，不注册 Effort，也不增加伪 transmission。
- 注册 RobotStateInterface 后，先加载 RobotStateController，由它创建 `robot_state` 句柄；底盘控制器随后加载。
- 两个 IMU 分别使用 `base_imu` / `gimbal_imu` 前缀加 `_quat`、`_gyro`、`_acc`。四元数由 MuJoCo wxyz 转为 ROS xyzw，传感器外参来自 `series_legged2`。
- 删除外部包中的 Hybrid 零值回退/延迟队列及 `RM_MUJOCO_TORQUE_TEST` 强制 1000 力矩测试钩子。当前轮腿使用 Effort 接口。
- JointState 的 effort 继承上游语义，是 `qfrc_applied` 读回，不是实测电机力矩、接触反力或该拍输出执行证明。
- 本地版本 `get_body_state` 的 pose header stamp 为零，检查脚本保留该字段并另记查询时的仿真时钟，不把查询时间伪装成源时间。
- 上游在 `mjcb_control` 阶段更新控制器。MuJoCo 的位置/速度传感器已更新，而加速度传感器在后续动力学阶段更新；同一次 readSim 中的 IMU 分量不能无条件称为同拍。此接入保留上游时序，没有通过重复推进物理或改控制器补齐相位。

<a id="dependencies"></a>

## 重建项目内依赖

引擎和 GLFW 下载来源：

- [MuJoCo 3.3.5 官方发布](https://github.com/google-deepmind/mujoco/releases/tag/3.3.5)，文件 `mujoco-3.3.5-linux-x86_64.tar.gz`。
- [GLFW 3.3.10 源码](https://github.com/glfw/glfw/tree/3.3.10)。

下载缓存位于工作区 `.local/downloads/`。MuJoCo 解压至 `.local/`；GLFW 源码解压至 `.local/build/`，构建命令为：

```bash
cmake -S .local/build/glfw-3.3.10 -B .local/build/glfw-build \
  -DCMAKE_INSTALL_PREFIX=/home/hzb/rm_ws/.local/mujoco-3.3.5 \
  -DBUILD_SHARED_LIBS=ON -DGLFW_BUILD_EXAMPLES=OFF \
  -DGLFW_BUILD_TESTS=OFF -DGLFW_BUILD_DOCS=OFF
cmake --build .local/build/glfw-build -j2
cmake --install .local/build/glfw-build
```

若改为从 `/home/hzb/Documents/mujoco` 编译引擎，遵循该 Noetic 分支 README 的 tinyxml2 固定选项 `-DMUJOCO_DEP_VERSION_tinyxml2=9a89766acc42ddfa9e7133c7d81a5bda108a0ade`。头文件和动态库必须来自同一安装。当前没有采用旧 Python 环境的 MuJoCo 3.2.3 动态库。

官方资料：[Noetic 分支 README](https://github.com/ubi-agni/mujoco_ros_pkgs/blob/noetic-devel/README.md)、[安装](https://ubi-agni.github.io/mujoco_ros_pkgs/devel/tutorials/getting_started/getting_started.html)、[模型和插件加载](https://ubi-agni.github.io/mujoco_ros_pkgs/devel/tutorials/load_custom_model/load_custom_model.html)。网站 devel 文档与仓库默认分支持续变化，接口以已选分支源码为准。
