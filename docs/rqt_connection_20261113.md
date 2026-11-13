# rqt 与 MuJoCo 命令链诊断（2026-11-13）

<a id="diagnosis"></a>

## 现场结论

只读检查用户正在运行的 `http://127.0.0.1:11321` 会话，未向该会话发送运动命令或切换控制器。rqt、MuJoCo、Manual 的 master 和 ROS IP 一致，实际 TCPROS 连接完整：

```text
rqt → /rm_ecat_hw/dbus → rm_manual
    → /cmd_chassis、/cmd_vel、/leg_cmd、云台 command → mujoco_server
```

两秒采样收到 203 条 DBUS、202 条底盘模式命令、201 条速度命令、200 条腿长命令和 200 条云台命令；仿真时钟正常推进，DBUS 与底盘命令时间戳对应。采样时输入居中，`s_l=3`、`s_r=3`（MID），`/cmd_chassis.mode=4`（FALLEN），控制器 Mode 为 `SIT_DOWN`。

原因来自 Manual 运行配置：

1. 现场诊断时，共享 [Manual YAML](../../rm_config/config/rm_manual/series_legged2.yaml) 的 `debug_gimbal_flag: true` 启用了仅调云台模式。[SeriesLeggedManual](../../rm_manual/src/series_legged_manual.cpp) 在构造时读取该参数，`updateRc()` 和 `updatePc()` 每拍将底盘命令强制为 FALLEN。因此控制器可以显示 running，机器人仍保持坐下。
2. 主启动文件当前已默认 `manual:=true`，但现场又运行了 `rosmon rm_config manual.launch`。该节点取代启动文件自带的同名 `/rm_manual`，其控制器清单重新包含 orientation、shooter 和 trigger calibration。现场已加载额外的 orientation，日志还出现 `left_friction_wheel_joint` 的 Effort 句柄缺失；当前仿真按用户要求没有该驱动。
3. 右拨杆中位是 RC 摇杆模式；键盘操作需要右拨杆上位和插件的 `Use keyboard`。这与 ROS 连接是否成立是不同条件。

现场原始日志位于工作区 `logs/mujoco_ros/4be6f0ae-df10-11f1-807b-4438e8b5b3af/`。`roslaunch-REDME-473786.log` 记录自带 Manual 在 11:15:10 退出；存活 Manual 的父进程为另行启动的 rosmon。没有连接、修改或停止用户的 11311 master。

<a id="fix"></a>

## 修复与使用

[原 Manual YAML](../../rm_config/config/rm_manual/series_legged2.yaml) 当前已设置 `debug_gimbal_flag: false`，作为唯一配置来源。最初为通信修复增加的 MuJoCo launch 同名参数和覆盖，已在后续配置精简中删除。需要仅调云台时修改原 YAML 的值。保留 `manual:=true` 默认值及底盘/云台四控制器的仿真配置。

配置精简涉及 `rm_mujoco` 的 launch、配置、测试入口和说明，以及原 Manual YAML 中未使用的字段和空校准列表。没有修改 Manual 源码、rqt 插件、控制算法参数或机械模型。

在原终端用 Ctrl-C 关闭另行启动的 `rm_config manual.launch` 及旧 MuJoCo launch，然后重新启动：

```bash
cd /home/hzb/rm_ws
source ./mujoco_env.sh
roslaunch rm_mujoco series_legged.launch manual:=true
```

rqt 使用同一环境；若已有 rqt 仍连到 11321，可继续使用。需要新开时执行：

```bash
cd /home/hzb/rm_ws
source ./mujoco_env.sh
rqt
```

Virtual Joystick 的 Topic 为 `/rm_ecat_hw/dbus`，开启底部发送开关。右拨杆中位用摇杆，上位用键鼠；键盘输入启用 `Use keyboard`。不用另外启动 `rm_config manual.launch`。若单独启动 master，命令为 `roscore -p 11321`；仅设置 `ROS_MASTER_URI` 不会改变不带 `-p` 的 roscore 所监听端口。

`debug_gimbal_flag` 是构造参数，只更改参数服务器不能更新已经运行的 Manual，必须重新启动节点。本次诊断保持用户正在运行的节点原状，上述重启后修复才会应用。

<a id="verification"></a>

## 验证结果

以下为配置精简前的通信修复验证。当时离线展开四种 launch 配置：默认、`debug_gimbal_flag:=true`、`manual:=false`、`manual:=false controllers:=false`，对应参数与节点清单全部通过。该同名 launch 参数现已删除，调试模式统一由原 Manual YAML 决定。Manual 模式仍只有一个 `/rm_manual`，直接命令模式使用 spawner，仅模型模式不加载这两者。

随后在新建 localhost 11329 master 上分别启动默认和仅调云台两组仿真。模型初始暂停，通过官方 step action 推进；只发布普通 DBUS 消息，未直接发布底盘/云台命令，也未修改控制器内部标志、姿态或接触。测试结束后关闭本次自有进程。

| 输入/检查 | 默认 `false` | 显式 `true` |
|---|---|---|
| 右拨杆中位，右摇杆前推 0.05 | FOLLOW；速度命令 0.11 m/s | FALLEN；速度命令仍可非零 |
| 右拨杆上位，W 按下 | FOLLOW；速度命令 2.2 m/s | FALLEN；速度命令仍可非零 |
| 摇杆/鼠标云台输入 | yaw/pitch 速率命令均有响应 | yaw/pitch 速率命令均有响应 |
| 右拨杆下位 | FALLEN、零速度、SIT_DOWN | FALLEN、零速度、SIT_DOWN |
| 控制器状态 | 四项 running | 四项 running |
| JointState | 16 个关节 | 16 个关节 |
| 仿真结束时间 | 0.342 s | 0.338 s |

默认组的兼容 Mode 消息出现 NORMAL，仅调云台组保持 SIT_DOWN。两组检查均通过，证明给定 DBUS 输入能经过原 Manual 到达 MuJoCo 控制器，并复现该调试参数导致的行为差异。

日志位于工作区 `logs/mujoco_integration/20261113_rqt_connection/` 的 `default/` 与 `gimbal_only/`，各有 `server.log`、`probe.log` 和 `result.json`。输入、命令和反馈分别记录，没有把异步样本拼为同拍数据。现场 rqt 发布链已经只读确认；隔离测试使用同格式 DBUS 发布器，没有自动点击 GUI。

本次是短时通信/操作模式检查，不是持续站稳、从坐姿起立、轨迹跟踪或实机验证。没有为修复通信问题调整控制器物理参数、Q/R 或 PID，无需重新编译 C++。

<a id="config-cleanup"></a>

## 后续配置精简验证

删除 MuJoCo 的调试参数覆盖、控制器 YAML 重复加载和重复状态控制器列表；将空云台校准列表归入原 Manual YAML，双 IMU 统一使用 `config/imus.yaml`。原 Manual YAML 另删除当前代码未读取的 20 个叶参数：腿部功率子树、顶层 `tof_topic`、五个旧 chassis 功率字段和拼写无效的 `use_local_heat_`。插件的默认命名空间/描述参数名及重复 `no_render` 转发也已省略。

对默认、`manual:=false`、`headless:=true`、`manual:=false controllers:=false` 四种入口做清理前后离线比较：除上述无效参数和显式默认值外，有效参数及节点清单一致。`rostest rm_mujoco complete_model.test` 使用独立测试 master，现有三项检查全部通过；未重新运行运动测试。当前配置修改无需重新编译 C++。长期运行的参数服务器可能仍保留已删除的旧键，重启隔离仿真 master 后可重新加载干净配置。
