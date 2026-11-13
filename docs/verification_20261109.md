# 2026-11-09 MuJoCo ROS 接入记录

<a id="scope"></a>

## 修改范围

用户授权按提供的 `mujoco_ros_pkgs`、`rm_mujoco` 和上游文档方式配置当前工作空间，并同步文档。新活动入口为 `roslaunch rm_mujoco series_legged.launch`；原控制算法、当前控制/Manual YAML、机械来源及已有 dirty 内容保留。

| 对象 | 本轮变化 |
|---|---|
| `src/mujoco_ros_pkgs` | 导入 Documents 中的 Noetic 版本。HEAD `aadba630`；未改上游运行实现。原副本的 sensors 测试 fixture 已是普通文件，与 Git 中 symlink 的类型差异一并保留。 |
| `src/rm_mujoco` | 导入原 package（源目录无 Git HEAD），适配轮腿状态、IMU、模型、launch 和配套检查。 |
| `RmMujocoRobotHWSim` | 增加八个被动关节的只读状态；沿用默认 effort 输出；删除 Hybrid 回退/队列和 1000 力矩环境测试钩子；修正协方差 YAML 整数的 XmlRpc 转换。 |
| `models/series_legged` | 保留来源 XML/URDF/清单；生成匹配六个驱动与固定上装的 ROS 描述；保留闭链、惯量、碰撞近似和源 effort 限值。 |
| `config/series_legged_*` | 官方插件配置、base_imu 参数、完整闭链初态，无算法参数覆盖。 |
| `launch/series_legged*.launch` | 包含官方服务器；原控制 YAML；状态控制器先加载；直接 topic 或原 Manual 两种方式。Manual 清单仅轮腿，并把 null 云台校准配置转为空列表。 |
| `.local/mujoco-3.3.5`、`mujoco_env.sh` | 项目内引擎/GLFW；当前终端的依赖路径与 localhost:11321 隔离环境。 |
| 文档 | 本包 README、模型说明、本记录；根 AGENTS、MuJoCo 入口、文档导航、ROS 框架入口及历史计划中的旧链接。旧平台 README/STATUS 随后按追加授权一并删除。 |

相对外部原包的已有文件改动见 [adapter_changes.patch](adapter_changes.patch)。版本、原仓库状态、模型和配置摘要见 [integration_manifest.json](integration_manifest.json)。没有提交或推送。

<a id="validation"></a>

## 实际验证

| 检查 | 结果与证据 |
|---|---|
| catkin 构建 | 相关 12 个 package 全部成功，包括 mujoco_ros_msgs、mujoco_ros、mujoco_ros_control、rm_mujoco、当前底盘控制器、RobotStateController、Manual 及依赖。[构建日志](../../../logs/mujoco_integration/20261109/catkin-build.log) |
| 引擎/GUI 依赖 | `ldd` 解析到工作区 `.local/mujoco-3.3.5/lib` 下的 MuJoCo 3.3.5 和 GLFW；硬件测试核对头文件版本与 `mj_version()` 相同。 |
| 硬件与模型测试 | 1 个综合回归用例通过；六路 effort、八个被动状态、IMU 姿态/传感器、四条闭链、总质量 20.41884 kg、全部初态坐标及闭合残差小于 1e-9 m、原 effort 限制及旧测试环境变量无效。[测试日志](../../../logs/mujoco_integration/20261109/hardware-test.log)、[XML 结果](../../../build/rm_mujoco/test_results/rm_mujoco/rosunit-hardware_interface.xml) |
| headless | 官方服务器及三项控制器成功加载，底盘取得六个 effort 资源；最终独立冷启动复核完成约 3.824 s 标准命令与 stop/start，`integration_passed=true`。[会话日志](../../../logs/mujoco_integration/20261109/ros-headless-final.log)、[检查结果](../../../logs/mujoco_integration/20261109/ros-smoke-headless.log)、[分流记录](../../../logs/mujoco_integration/20261109/ros-smoke-headless.json) |
| GUI | 当前 DISPLAY 上实际存在 `MuJoCo ROS Viewer : series_legged_complete_frozen_upper` 窗口。[窗口检查](../../../logs/mujoco_integration/20261109/gui-check.json)、[会话日志](../../../logs/mujoco_integration/20261109/ros-gui.log) |
| GUI 会话 ROS 集成 | 正式检查约 3.824 s 仿真；14 个关节反馈，382 条 JointState、369 条兼容底盘状态、190 次模型状态查询；普通命令及底盘 stop/start 均通过。[脚本结果](../../../logs/mujoco_integration/20261109/ros-smoke.log)、[分流记录](../../../logs/mujoco_integration/20261109/ros-smoke.json) |
| Manual 初始化 | 节点运行；joint_state/robot_state 为 running；底盘为 initialized 等待输入；未加载云台/发射/校准控制器。[检查结果](../../../logs/mujoco_integration/20261109/manual-check.json)、[日志](../../../logs/mujoco_integration/20261109/ros-manual.log) |
| 原有工作保护 | controllers/config/control/description 四仓的工作区 diff 与接入前保存内容完全一致，未切换分支。文档仓保留原修改，仅增补导航与接入说明。 |
| 文件检查 | 新 launch/XML/YAML 和 Python 语法检查；新适配 C++ 按现有 clang-format 排版。未运行 clang-tidy，不宣称通过。 |

<a id="behavior"></a>

## 命令、模式与物理结果的边界

正式检查从闭链一致的 0.20 m 预设构型开始，发送 RAW 零速约 3 s、RAW 0.1 m/s 约 0.5 s、FALLEN 约 0.2 s，随后只停止/启动底盘控制器并继续 FALLEN 约 0.1 s。

观察到 `SIT_DOWN → STAND_UP → NORMAL → SIT_DOWN`。底盘在 manager 中从 running 变为 stopped，再变为 running，两项状态控制器保持 running。Mode 消息没有源时间戳，记录的是独立回调接收时的仿真时间，不与异步状态拼成同拍数据。

MuJoCo 模型状态查询在仿真时钟约 1/2/3 s 时，基座高度约 0.2093/0.2106/0.2118 m，x 约 -0.00003/0.03325/0.07296 m。零速阶段仍有位移，腿长/高度也不等于初始摆放值；因此此短检查不能解释为精确悬停或轨迹跟踪验收。它证明了当前插件、命令和物理引擎已经接通。

FALLEN 后高度下降属于零力矩后自然演化。测试没有通过服务扶正、改接触或写内部动作标志。初态已经接近站立，瞬时经过 STAND_UP 不能证明从倒地状态物理起立成功。

底盘状态和 JointState 是兼容反馈；JointState effort 为上游 `qfrc_applied` 读回。实际施力路径和饱和由独立硬件用例直接核对，不能把异步读回值等同于本拍执行证明。`get_body_state` 的源 header stamp 在本版本为零，原样记录为 `service_stamp`，查询时钟另列 `query_after_step_sim_time`。

<a id="limits"></a>

## 未覆盖与保留问题

- 未做从倒地状态的冷启动起立、UPSTAIRS 越障、跳跃、全腿长域或长期稳定性验收，也未调参。机械模型不含弹簧，而控制器仍保留当前源码中的补偿，模型适用范围按模型说明解释。
- Manual 原 YAML 中的 `gimbal_calibration: null` 导致 ROS 参数上传失败，已在仿真 launch 中覆盖为空列表，原文件未改。原框架还报告 `disable_burst_cap_threshold`、`enable_burst_cap_threshold`、`disable_normal_cap_threshold`、`enable_gyro_cap_threshold`、`disable_gyro_cap_threshold`、`gyro_power`、`upstairs_power`、`max_power_limit`、`follow_source_frame`、`chassis_motor` 缺项。消费者位于 `rm_common/decision/power_limit.h`、`command_sender.h` 及 `rm_manual/chassis_gimbal_shooter_cover_manual.cpp`。未猜测参数或借用其他车型填充；完整遥控器/键鼠链未验证。
- 原 RobotStateController/KDL 提示根 link 惯量不受 KDL 支持，原动态配置服务提示互斥锁用法；本次未修改这些公共模块。MuJoCo 使用自身模型惯量，模型测试单独核对质量。退出会话时 spawner 曾提示服务器已退出导致卸载服务中断，实际模拟进程随后全部结束。
- 上游 IMU 加速度与位置/速度传感器的更新阶段不同，保留原回调时序；详见本包 README。
- 未构建或测试上游可选的 camera/laser/mocap 功能；导入包中其他车型资源保留，未验证。
- 仿真 master 始终为独立 localhost:11321，硬件 rostest 自建另一个隔离本地 master；没有连接继承环境中的 `192.168.1.102:11311`。验证结束关闭本次启动的 master、服务器和 Manual。

<a id="attempts"></a>

## 保留的失败尝试

调试检查脚本时，首次在暂停状态下先查询 manager 服务，遇到待完成的 start 持有服务锁；额外 step 客户端又与脚本 action 冲突。脚本已改成先步进直到收到控制器反馈、再查询，并只保留一个步进客户端。后续一次会话重启与自动 master 退出发生竞争，改为等前一会话完全退出再启动，或单独 roscore 贯穿检查。脚本也曾将 SwitchController 的 float timeout 误传为 Duration，已修正为 float。最后的 headless 复核发现 actionlib 的 SUCCEEDED 状态可能先于结果消息到达；检查脚本现等待实际 result，避免将尚未收到结果误判为步进失败。

这些属于检查流程的问题，没有通过修改控制算法处理。相关 `*-startup-attempt*`、`*-master-restart-attempt*`、`*-switch-type-attempt*`、`*-action-result-attempt*` 日志/数据保存在 [日志目录](../../../logs/mujoco_integration/20261109)。Manual 的 null 参数失败记录为 `ros-manual-null-calibration-attempt.log`。最终成功结果见上表，不覆盖失败证据。

<a id="legacy-removal"></a>

## 追加清理：旧平台删除

2026-11-09 按用户“原平台若已无用，删去且不留备份”的明确授权，删除整个工作区根目录 `mujoco_platform/`（含其中的历史实验、运行器及构建产物）和仅调用其 `lab.py` 的旧 `mujoco` 脚本。未创建备份。

删除前确认无进程占用、无目录外符号链接指入旧平台；当前 launch、模型生成、硬件适配及构建配置均不依赖旧目录。新 package 内的机械来源为当前生成器的实际输入，独立文件保留在本 package。删除未沿旧平台内的 Python 环境符号链接处理其他目录。

删除后只读核对 `rospack find rm_mujoco`、直接/Manual 两种 launch 展开、MuJoCo/GLFW 动态库路径及模型文件完整性；本次清理没有重建或重新启动仿真。历史文档中已删除报告的链接改为明确的删除说明，旧记录不替代当前运行证据。
