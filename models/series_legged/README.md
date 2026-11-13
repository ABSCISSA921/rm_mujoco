# series_legged 机械模型的 ROS 接入

<a id="source"></a>

## 来源和生成

`source/` 保留从旧平台迁入的三份机械来源文件：`series_legged.xml`、`expanded_source.urdf` 和 `manifest.json`。源清单中的旧路径、参数摘要和核对日期是历史信息，不代表当前运行的控制参数。

生成当前 ROS 文件：

```bash
python3 src/rm_mujoco/scripts/prepare_series_legged.py src/rm_mujoco/models/series_legged
```

脚本只使用 package 内来源文件。它移除 MJCF 的六个 motor（Noetic 默认硬件通过 `qfrc_applied` 施力），将 IMU 传感器改为 rm_mujoco 的命名约定；在 ROS URDF 中固定模型没有保留的活动关节，移除其 transmission 和 Gazebo 扩展。六个主动关节的 URDF effort 限值保持腿关节 80 Nm、轮关节 3.58 Nm，由上游默认硬件接口执行。

没有重新拟合机械参数、重新转换 CAD 或改源 URDF。`robot.urdf` 的视觉网格通过当前 `rm_description` package 查找；`scene.xml` 自身采用已保存的几何，不依赖旧平台或 Documents 目录。

<a id="model"></a>

## 保留的机械定义

- 总质量 20.41884 kg，35 个源 link 的质量和惯量保留。非轮腿上装冻结于源零位，聚合惯量不变。
- 六个主动铰链、八个被动铰链、一个自由基座 `base_free`、四条闭链 connect 约束。
- VMC 编码器约定保持 hip + π、knee，不重复施加减速比或硬件 offset。
- 驱动轮半径 0.06 m；原 mesh 碰撞的包围盒近似、机器人与地面碰撞及关闭自碰撞的设置保留。
- 不含机械弹簧。当前控制器的弹簧补偿按原源码运行；这里没有为仿真增加算法参数覆盖。
- 物理步长 0.001 s，implicitfast、Newton、100 次迭代，重力 `[0,0,-9.81]`。
- IMU 外参 xyz=`[0.15962,0,-0.029125]`，rpy=`[-π,0,-π/2]`。

<a id="initial"></a>

## 初始构型

[初态 YAML](../../config/series_legged_initial.yaml) 保存自由基座和全部十四个铰链的位置、零初速度。构型从已有闭链求解器一次性导出，目标轮心距离 0.20 m，基座高度约 0.2602 m，原始闭链点残差约 3.57e-15 m。导出环境记录于 `initial_state.json`；运行只需 YAML，不依赖该 Python 求解器。

这是接触前的初始摆放，不是物理起立成功或稳定保持证据。启动后只通过控制器力矩和 MuJoCo 动力学演化。改变初始腿长时必须提供一致的被动关节构型，不能只修改两个主动编码器值。

实际在 MuJoCo 3.3.5 中的模型、接口和运行核对见 [验证记录](../../docs/verification_20261109.md)。
