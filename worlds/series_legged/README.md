# series_legged 世界资源

从 [Gazebo启动入口](../../../rm_control/rm_gazebo/launch/series_legged_world.launch)实际引用的`series_legged.world`迁入，原Gazebo文件保留。运行时地形、网格和模型均位于`rm_mujoco`，不依赖Gazebo资源路径。

| 文件 | 用途 |
|---|---|
| [terrain.xml](terrain.xml) | 可复用地形MJCF，包含原视觉网格及26个凸碰撞块 |
| [scene.xml](scene.xml) | 只显示世界：地形、地面和方向光 |
| [meshes/series_legged_world.stl](meshes/series_legged_world.stl) | 原始STL逐字节副本，172个三角面 |
| [source/series_legged.world](source/series_legged.world) | 原SDF逐字节副本，仅作来源记录 |
| [conversion.json](conversion.json) | 原点偏移、范围、体积及转换方式 |
| [机器人＋世界](../../models/series_legged2/series_legged_world.xml) | 复用当前完整机器人和平地地面，再加入地形 |

地形保持源世界的平移`(-2, -0.6, 0)m`、单位比例和方向，范围约为x`[-7,9.305]m`、y`[-1.734,7.043]m`，最高点约`.6266m`。世界原点`(0,0)`位于高`.20m`的平台上。

MuJoCo普通网格碰撞使用凸包，整份STL直接碰撞会填平凹槽，见[MuJoCo3.3.5碰撞说明](https://mujoco.readthedocs.io/en/3.3.5/computation/index.html#checking)。本资源沿源三角面平面拆分内部体积，再合并凸邻块，不使用体素高度栅格。合计26个凸块，体积与源模型相差约`3.72e-8 m³`；共面合并容忍约`1e-6`。视觉顶点和面保持源数据，内嵌MJCF以免受机器人`compiler.meshdir`影响。

碰撞使用`contype=2/conaffinity=1`，与当前机器人掩码对应；`condim=3`，摩擦`1 .005 .0001`及接触软硬参数沿用当前MuJoCo平地。源SDF没有显式指定摩擦数值，这不代表逐项复刻ODE求解器。碰撞位于group3，默认查看原STL外观；打开group3可显示半透明碰撞块。

## 启动

继续使用当前分终端rosmon／virtual_dbus链。每个终端先加载环境，master仍使用隔离localhost端口：

```bash
source /home/abscissa/rm_ws/mujoco_env.sh
```

仿真终端执行：

```bash
mon launch rm_mujoco series_legged_world.launch
```

另一个终端执行：

```bash
mon launch rm_config manual.launch
```

再以`rqt --standalone rqt_virtual_dbus/MyPlugin`操作。环境脚本已设置`ROBOT_TYPE=series_legged2`。世界入口默认`manual=false/controllers=false`，由单独启动的Manual加载和管理控制器。仍可使用`manual:=true`改为内部启动Manual，此时只需启动世界入口。

配套初态为 [series_legged2_world_initial.yaml](../../config/series_legged2_world_initial.yaml)：仅将原base z从`.2602`抬到`.4602m`，其他关节与姿态不变，以适配原点的平台。只给旧入口覆盖`modelfile`而保留平地初态会使轮子嵌入平台，应使用新入口。原`series_legged.launch`继续加载原平地场景。

只查看世界、不放机器人：

```bash
roslaunch mujoco_ros launch_server.launch \
  modelfile:="$(rospack find rm_mujoco)/worlds/series_legged/scene.xml"
```

## 再生成与验证

源Gazebo资源更新后，在工作区执行`python3 src/rm_mujoco/scripts/prepare_series_legged_world.py`重新生成地形和来源副本；脚本需要现有numpy/scipy。机器人模型和控制算法不由此脚本修改。

本次四项[资源测试](../../tests/series_legged_world_test.cpp)通过：独立世界加载/碰撞掩码、源三角面位置的碰撞高度和凹槽、机器人动力学/弹簧参数保持、球体实际落在平台上。新launch已展开核对，并在独立localhost11322无控制器加载后成功执行5个物理步；初态使用`.4602m`。这次验证覆盖世界接入，未执行越障或控制器动作验收。
