# series_legged2：完整 CAD 的 ROS 仿真适配模型

<a id="sources"></a>

## 来源与坐标

2026-11-13 按用户确认组合三份模型。生成器为 [prepare_series_legged2.py](../../scripts/prepare_series_legged2.py)，输入摘要与每个刚体的 CAD→ROS 变换保存在 [source_manifest.json](source_manifest.json)。

| 内容 | 来源 |
|---|---|
| 腿部连杆尺寸、关节原点/轴、两处闭链连接、轮距与轮半径 | `rm_description/urdf/series_legged` |
| 腿部关节限制；云台 yaw/pitch 安装、关节定义；base/gimbal IMU 安装位置 | `rm_description/urdf/series_legged2` |
| base/gimbal IMU 安装方向 | 保留名义安装方向，去掉小角度安装偏置：base 为 `(0, π, 0)`，gimbal 为 `(0, 0, −π/2)` |
| 八路 transmission，包括 chassis 宏调用中的 reduction/offset | `rm_description/urdf/series_legged2` |
| 17 个刚体的质量、质心、惯量、完整网格及碰撞包络 | [serial_leg_complete](../serial_leg_complete/README.md) |
| 每侧弹簧的等效轴向力 | 用户提供的三次拟合曲线 |
| 每侧髋轴到轮心的最大平面距离 0.40 m | 用户指定的机械距离上限 |

原始 CAD 的 `left` 髋位于 −Y。适配采用绕 Z 轴旋转 π 的正常刚体旋转，使其成为 ROS 的 +Y 左腿；base 原点移至髋轴中点。各杆再依据铰点配对转换到 URDF 连杆局部坐标。质心按完整位姿变换，绕质心的惯量按 $I'=RIR^T$ 变换，保留各刚体质量与惯量主值。yaw/pitch 网格及惯量局部轴同样绕 Z 轴旋转 π，安装变换采用 `series_legged2`。

CAD 长杆两端间距与指定 URDF 的 0.20 m 相差约 **0.50961 mm**；运动学采用 URDF，网格不拉伸、惯量不缩放，因此该处外观铰点存在相应小偏差。轮子碰撞半径采用 URDF 的 **0.06 m**、轮心距 **0.446 m**；保留原 CAD 外观网格，其轮胎外观半径约 0.06101 m。

原 `series_legged` 的圆整角度和闭链几何与当前控制器的等效五连杆也有小差异：在本模型实际垂直腿长 0.20 m 的初始姿态，现有 VMC 得到约 **0.198175918 m、−0.006483476 rad**。本次按用户要求保留指定几何和原控制器参数，没有通过改反馈数值消除这个差异。

<a id="interfaces"></a>

## 关节与物理环境

- 六路底盘和 yaw/pitch 共 **8 个 EffortJoint**；另有 8 个被动连杆关节，共 16 个关节状态。
- `hip_joint` 驱动大腿；`knee_joint` 驱动与髋同轴的 output 杆。两者父链接均为 `base_link`；`calf_joint` 为被动膝关节。
- 髋/膝驱动限制为 **40 N·m、10.46 rad/s**；pitch 位置范围为 **[−0.60, 0.30] rad**。限值来自 `series_legged2`。没有沿用 CAD 预览模型的被动关节范围和相对角限位 tendon。
- 传动数据完整保存在 [robot.urdf](robot.urdf)。当前官方 `DefaultRobotHWSim` 接收的是关节空间命令，直接写入 `qfrc_applied`；不再次乘 reduction 或加硬件标定 offset。模型没有另一组 motor actuator。
- base/gimbal IMU 各有 quaternion、gyro、accelerometer。XYZ 与源 `series_legged2` URDF 一致；base IMU 的 RPY 由 `(0.00188, π + 0.00733, 0)` 简化为 `(0, π, 0)`，gimbal IMU 保留 `(0, 0, −π/2)`。MJCF 与本目录供 ROS TF 使用的 URDF 同步采用这些名义安装方向，[IMU 配置](../../config/imus.yaml) 使用相同的 frame 名称；源 URDF 保留实机安装偏置。
- 拨弹和摩擦轮不单独驱动，其 CAD 质量与外观已经包含在相应刚体中；辅助 ROS frame 不重复增加质量。
- 浮动基座、四个闭链 connect、重力 −9.81 m/s²、平地接触、0.001 s 物理步长。碰撞采用 CAD 近似包络，机器人自身不同碰撞体之间不启用自碰撞。
- 左右腿各有一个 `spatial tendon`，复用 `*_spring_hip` 和 `*_spring_wheel` 两个端点，以 `limited="true" range="0 0.40"` 限制髋轴到轮心的平面距离（不含轮半径）。0 是距离的自然下界，没有额外规定最小腿长。限位由 MuJoCo 约束求解器产生反力；区间内部不增加弹簧力或阻尼。`solreflimit="0.002 1"`、`solimplimit="0.999 0.9999 0.0001"` 定义限位的数值柔度，受载或冲击时允许少量越界。该距离上限不能单独保证六连杆始终处于正确装配分支。参见 [MuJoCo tendon 限位](https://mujoco.readthedocs.io/en/3.3.5/XMLreference.html#tendon-spatial)。
- 初始腿长为 0.20 m，轮胎距地面 0.2 mm，yaw 关节为 π/2，使云台面向 base +X，pitch 为 0。初始姿态不代表完成起立。

<a id="spring"></a>

## 被动弹簧

每侧以髋轴到轮轴的平面距离 $L$（m，不含轮半径）计算：

$$
F(L)=2094.45L^3-3091.28L^2+1408.375L-80.91\quad\mathrm{N}.
$$

正值表示伸腿力。两个作用点沿腿方向施加大小相等、方向相反的内部力，经 `mj_applyFT` 累加到 `qfrc_passive`，不受电机力矩限制裁剪，也不随控制器停止而消失。实现见 [leg_spring.cpp](../../src/leg_spring.cpp)。这是用户曲线给出的**等效轴向弹簧**，未新增阻尼或猜测真实弹簧安装点。

系数保存在 MJCF `custom/numeric`，由 [LegSpringPlugin](../../src/leg_spring_plugin.cpp) 在官方被动力回调读取并应用。必须通过 [series_legged.launch](../../launch/series_legged.launch) 加载对应插件配置；仅用原生 MuJoCo 打开 `scene.xml` 不会自动执行这个 ROS 插件。调用约定见 [MuJoCo 被动力说明](https://mujoco.readthedocs.io/en/3.3.5/computation/index.html#passive-forces) 和 [mj_applyFT](https://mujoco.readthedocs.io/en/3.3.5/APIreference/APIfunctions.html#mj-applyft)。

控制器原来的 `-f_spring_force(L0)` 补偿保持原样。仿真弹簧代表机械作用，补偿属于电机命令，二者职责不同。

<a id="usage"></a>

## 使用与再生成

活动入口为 `source ./mujoco_env.sh` 后运行 `roslaunch rm_mujoco series_legged.launch`；Manual 使用 `manual:=true`。原完整副本和 `serial_leg_complete.launch` 仍用于查看固定底座、零重力原始 CAD。

控制器继续读取唯一的 `rm_config/config/rm_controllers/series_legged2.yaml`，本次未同步其物理参数、Q/R、PID。该设置包含用户明确保留的模型参数失配。

仅在需要更新机械时手动再生成：

```bash
source ./mujoco_env.sh
python3 src/rm_mujoco/scripts/prepare_series_legged2.py
```

生成器写入本目录的 MJCF、URDF、来源记录，以及 `config/series_legged2_initial.yaml`；不写源 URDF、原 CAD 副本或算法配置。验证范围与结果见 [2026-11-13 记录](../../docs/verification_20261113.md)。
