# 导航栈设计文档（car_navigation：AMCL + Nav2）

日期：2026-10-13
主题：在已建好的离线地图上落地导航层（定位 AMCL + 规划 + 控制），控制器 DWB/MPPI 可切换
状态：设计已定稿，实施中（落地包 `car_navigation`；§11 验收记录待回填）
相关文档：`../sim_car_spec.md`（车体规格/坐标约定/踩坑）、`../FAQS/ros2-launch.md`（launch 同名参数坑）、
`docs/devel/2026-10-13-plan-navigation-stack.md`（实现步骤与逐项落地记录）

## 1. 背景与目标

建图链路已完成：`src/car_slam/maps/local_map.pgm|yaml` 是 8m×6m 家庭场景（左客厅 / 右上卧室 /
右下厨房）的离线栅格图。现在要补上「用这张图定位 → 规划 → 控制到目标点」这一层，即
移动 → 定位 → 建图 → **导航** 链路的最后一环。

目标：

- 新增第 5 层**导航层** `car_navigation`（纯配置包 + 一个航点回归脚本），与 `car_slam` 同构。
- 复用既有层：场景/机器人/定位经 `car_bringup/gazebo.launch.py` 接口（launch args）引入，
  map_server + amcl 经 `nav2_bringup/localization_launch.py` 接口引入。
- 控制器两档可切换（DWB / MPPI），为后续 TEB 预留同一接口。
- 与通信层（`car_route` / Qt 面板）**物理隔离**：导航模式下 `/cmd_vel` 由 nav2 独占。

不做（见 §12）：TEB（无 Humble 二进制包）、Qt 面板下发导航目标（协议扩展）、实物联调。

## 2. 分层与 TF / QoS 契约

```
导航层  car_navigation（launch + config + 航点脚本）
         ├─ 复用 car_bringup：世界 + 机器人 + car_localization(EKF)
         ├─ 复用 nav2_bringup/localization_launch.py：map_server + amcl
         └─ 自建 nav2 导航节点列表（见 §4）
```

TF 链（**每个变换仍只有一个发布者**，与建图模式二选一，不得同时启动）：

| 变换 | 建图模式 | 导航模式 |
|---|---|---|
| `map→odom` | slam_toolbox | **AMCL** |
| `odom→base_footprint` | EKF（`car_localization`） | EKF（同左，不变） |
| `base_link→传感器/轮` | robot_state_publisher | 同左 |

话题与 QoS（AGENTS.md 冻结契约不变）：

- `/cmd_vel`（Twist, reliable depth1）：手动模式发布者是 `translator`，导航模式是
  `velocity_smoother`。两者互斥（§8）。
- `/scan`（SensorDataQoS）；`/odom`、`/wheels/downlink`（best_effort depth10）；
  `/wheels/uplink`（reliable depth10）。
- nav2 内部：controller 发 `cmd_vel_nav` → velocity_smoother 发 `/cmd_vel`
  （remap 与 `nav2_bringup/navigation_launch.py` 完全一致，不引入第二套约定）。
- `map_server` 的 `/map` 为 transient_local（RViz 端 Map 显示项必须配 Transient Local）。

## 3. 定位方案：AMCL + 离线地图

**为什么是 AMCL**：现有产物只有 `.pgm/.yaml`（静态栅格图），没有 slam_toolbox 的序列化位姿图
（`.posegraph`/`.data`），因此 slam_toolbox 的 `localizing` 模式无从加载；AMCL 正是「栅格图 +
激光 + 里程计」的标准定位器。

**初始位姿可用 (0,0,0) 的核算**（关键前提，若换建图起点必须重算）：

| 依据 | 数据 |
|---|---|
| 地图尺寸 | 159×120 px @ 0.05 m/px → 7.95m × 6.0m |
| 地图 yaml origin | (−0.984, −0.99) |
| 地图占用区（map 帧） | x ∈ [−0.984, 6.966]，y ∈ [−0.99, 5.01] |
| 世界墙位 | x = ±4，y = ±3（`home_world.sdf`） |
| 反推 map 原点在世界系的位置 | (−4, −3) 落在 map 帧 (−1, −1) → **map 原点 = 世界 (−3, −2)** |
| 建图起点 | `slam.launch.py` 默认 spawn (−3,−2)，yaw 0 |
| diff_drive odom 原点 | 编码器积分的起点 = spawn 位姿（`odometry_source=0`） |

结论：**map 原点 = odom 原点 = 建图起点**，故 AMCL 初始位姿 (0,0,0) 即为真值附近，
`set_initial_pose: true` 直接生效，无需每次手点 2D Pose Estimate（但保留该手段做校验）。

推论（写进 launch docstring 与 FAQ）：**导航必须从建图起点出生**（`spawn_x/y/yaw` 默认 −3.0/−2.0/0.0）。
若改了出生点，map 与 odom 的关系不再简单可推，须用 RViz 的 2D Pose Estimate 重新给定位姿。

## 4. 接口与复用

### 4.1 对外 include（接口 = launch args）

| 被 include | 传入参数 | 目的 |
|---|---|---|
| `car_bringup/gazebo.launch.py` | `world/gui/with_camera/use_sim_time/spawn_x/y/yaw`、`rviz:=false` | 世界 + 机器人 + EKF 定位栈 |
| `nav2_bringup/localization_launch.py` | `map`、`params_file`、`use_sim_time`、`autostart:=true` | map_server + amcl + 其生命周期管理 |

`rviz:=false` 是必须的：本包自带导航 RViz，且 `gazebo.launch.py` 的 `rviz` 参数与顶层重名会静默覆盖
（`docs/FAQS/ros2-launch.md` 的坑），故本包顶层参数取名 `nav_rviz`（与 `car_slam` 的 `slam_rviz` 同理）。

### 4.2 为什么自建导航节点列表

`nav2_bringup/navigation_launch.py` 只接受**单一** `params_file`，其内部 `RewrittenYaml` 仅重写
`use_sim_time`/`autostart` 两个键（已读源码确认：`RewrittenYaml.substitute_params` 只替换**已存在**的
叶子键）。而控制器切换需要按 launch 参数换掉 `controller_plugins` 列表与对应的控制器参数段——
DWB 与 MPPI 的参数名有大量冲突（`max_vel_x` 两边含义不同），**不可能共存于同一 `FollowPath` 段**。

因此本包自己在 launch 里列 nav2 的 8 个节点（controller_server / smoother_server / planner_server /
behavior_server / bt_navigator / waypoint_follower / velocity_smoother / lifecycle_manager），
**结构、remap、lifecycle 顺序与 nav2 原版逐条对齐**，仅在 `controller_server` 的参数表末尾追加一个
按 `controller` 参数选中的控制器文件：

```python
parameters=[navigation_params,
            PathJoinSubstitution([nav_share, 'config',
                                  ['controller_', LaunchConfiguration('controller'), '.yaml']]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}]
```

多参数文件后者覆盖同名键是 rcl 标准行为（后者在 `--params-file` 中靠后）。这是本包唯一偏离
「层间只经 launch 接口复用」之处，理由记录在此。

## 5. 参数设计

### 5.1 footprint（由 URDF 实测推出，非估摸）

`docs/sim_car_spec.md` §3 + `car.urdf.xacro` 实测外廓：

| 方向 | 来源 | 值 |
|---|---|---|
| −x（车尾） | 驱动轮中心 x=−0.05075 − 轮半径 0.0325 | −0.0833 |
| +x（车头） | 前万向轮 x=0.0652 + 球半径 0.0109（底盘前碰撞盒只到 0.0723） | +0.0761 |
| ±y（车宽） | 轮心 ±0.064 ± 半轮宽 0.017 | ±0.081 |

取整外扩到 `[[-0.085,-0.082],[-0.085,0.082],[0.080,0.082],[0.080,-0.082]]`
（**必须写成字符串**：nav2 的 `footprint` 参数声明为 string，嵌套列表不是合法参数类型）。
不用 `robot_radius`：本车近方形（0.165×0.164），外接圆半径 0.12 会白白吃掉门洞净宽。

### 5.2 速度包络（对齐 spec §4：v ≤ 0.35 m/s，ω ≤ 2.0 rad/s）

| 位置 | 参数 | 值 | 理由 |
|---|---|---|---|
| DWB | `max_vel_x` / `max_vel_theta` | 0.30 / 1.5 | 留 15% 余量给控制器 |
| DWB | `acc_lim_x` / `acc_lim_theta` | 0.2 / 0.8 | 见 §7.2：底盘实际线加速度仅 ~0.1 m/s²、角加速度 ~0.76 rad/s² |
| DWB | `vx_samples` / `vtheta_samples` / `vy_samples` | 20 / 40 / 1 | 非全向，`vy` 恒 0 |
| DWB | `sim_time` | 1.7 s | 1.7×0.3 ≈ 0.5m 前视，够 3×3m 局部窗口 |
| goal_checker | `xy` / `yaw` | 0.10 / 0.20 | 车小、房间小，比 nav2 默认（0.25/0.25）更严 |
| progress_checker | `required_movement_radius` / `movement_time_allowance` | 0.10 / 15.0 | 起步慢，放宽时间避免误判卡死 |
| velocity_smoother | `max_velocity` / `min_velocity` | [0.30,0,1.5] / [−0.15,0,−1.5] | 反向留给 recovery 的 BackUp |

### 5.3 AMCL

`scan_topic: scan`、`odom_frame_id: odom`、`global_frame_id: map`、`base_frame_id: base_footprint`、
`robot_model_type: DifferentialMotionModel`、`set_initial_pose: true` + `initial_pose` 全 0（§3）、
`max_beams: 90`、`update_min_d/a: 0.10`（车慢，比默认 0.25/0.2 更密）、`tf_broadcast: true`、
`laser_max_range/min_range: -1.0`（取雷达自带 0.10–12m）、粒子 2000/500（8×6m 小图，够用且便宜）。

### 5.4 两级代价地图

| 项 | local | global |
|---|---|---|
| frame | `odom`（rolling 3×3m） | `map` |
| `robot_base_frame` | `base_footprint` | `base_footprint`（nav2 默认 base_link，**必须改**） |
| `resolution` | 0.05（与地图一致） | 0.05 |
| 插件 | obstacle_layer(scan) + inflation | static_layer(transient_local) + obstacle_layer + inflation |
| inflation | `radius 0.25` / `scaling 3.0` | 同左 |
| 其他 | `always_send_full_costmap: true` | `track_unknown_space: true` |

膨胀半径取 0.25 的约束核算：门洞 0.8m − 两侧各 0.25m = 0.3m 净宽 > 车宽 0.165m，可通过；
全局规划器 NavFn `allow_unknown: true`（未知空间不致命，小图内含在墙内，不封死规划）。

### 5.5 MPPI（第二档）

`motion_model: DiffDrive`、`batch_size 2000`、`time_steps 56`、`model_dt 0.05`、
`vx_max 0.3 / wz_max 1.5`、critics 采用官方 DiffDrive 模板
（Constraint/Cost/Goal/GoalAngle/PathAlign/PathFollow/PathAngle/PreferForward）、
`visualize: true`（发布 `/trajectories`，RViz 看采样轨迹便于调权重）。
critic 权重先用 nav2 文档模板值，**首跑后按实测调**（记入 §11）。

## 6. 控制器切换机制

`controller:=dwb|mppi`（默认 dwb）→ 选中 `config/controller_<name>.yaml`，与
`config/nav2_navigation.yaml` 一起传给 `controller_server`（后者覆盖前者同名键）。
每个控制器文件自包含：`controller_plugins: ["FollowPath"]` + 同名 `FollowPath` 段（含 `plugin` 类名）。

TEB 待补的接入点明确：源码编译出 `teb_local_planner` 的 nav2 插件后，只需新增
`config/controller_teb.yaml`（同一结构），launch 与 BT 均无需改动。
（已核实本机 USTC 镜像的 Humble 二进制源无 `ros-humble-teb-local-planner`，而 dwb/mppi/rpp 都有。）

## 7. 启动时序与停车兜底

### 7.1 必须延后启动 nav2

gzserver 加载世界 + `spawn_entity.py` 完成前，`/clock` 未发布、`odom→base_footprint` 不存在；
此时 nav2 生命周期节点 configure 会拿不到时钟/TF 而失败，autostart 随即放弃使整栈停在 inactive。
故定位栈与导航栈整体包在 `TimerAction(period=nav_start_delay)`（默认 15s，`sim:=false` 时建议置 0）。
动作顺序：bringup → 导航 RViz → TimerAction(定位栈 → 导航栈)。

### 7.2 停车兜底（补既有坑）

`car.gazebo.xacro` 的 `diff_drive` 插件**没有 `cmd_vel_timeout`**：指令停发后目标速度保持
（spec §4 已记录）。手动模式由 `translator` 的 500ms watchdog 发全零兜底；导航模式没有任何节点兜底
——除非利用 `velocity_smoother` 的 `velocity_timeout`：

> nav2 README 原文：*"Time (s) after which if no new velocity commands are received to zero out and stop"*

本设计取 `velocity_timeout: 0.5`（与 translator 的 500ms watchdog 对称），因此**无需自写 watchdog 节点**。
注意其边界：该兜底只覆盖「controller 不再发指令」，若 velocity_smoother 自身被杀，仍会保持末次速度。

### 7.3 起步能力的自查项

`max_wheel_acceleration: 3.0 rad/s²` 折算线加速度 ≈ 0.0975 m/s²（3.0 × 0.0325），到 0.3 m/s 需 ~3s；
角加速度 ≈ 0.76 rad/s²。DWB 的 acc 限值即按此设定。若实测表现为跟不上路径/进度检查超时，
优先级 1 是提高该 URDF 参数（仿真侧电机能力假设，须同步 spec §4），优先级 2 才是放宽 DWB 限值。

## 8. 与通信层（car_route）的互斥

**物理隔离，不做仲裁**。原因：`translator` 在无面板连接时会由 500ms watchdog 每 100ms 发全零
`Twist` 到 `/cmd_vel`，与 nav2 的指令直接对打（同话题互相覆盖），表现为车走走停停。
`twist_mux` 未安装，且引入它意味着改动通信层拓扑与新增依赖，超出本轮范围。

约定（写进 AGENTS.md 与 launch docstring）：**导航模式不启动 `route.launch.py`；需要 Qt 遥控时用
`car_bringup/gazebo.launch.py` + `route.launch.py`，不要叠加导航栈。**
后续若要「面板下发导航目标」，属 `car_route` 信封协议扩展，另开设计文档。

## 9. 文件清单

| 文件 | 职责 |
|---|---|
| `launch/navigation.launch.py` | 总入口：可选 Gazebo + EKF + AMCL/map_server + nav2 八节点 + 导航 RViz |
| `config/nav2_localization.yaml` | `map_server`（含必须显式写出的 `yaml_filename: ""`）与 `amcl` |
| `config/nav2_navigation.yaml` | controller_server（不含控制器段）/planner/smoother/behaviors/bt_navigator/waypoint_follower/velocity_smoother + 两级 costmap |
| `config/controller_dwb.yaml` | DWB 档：`controller_plugins` + `FollowPath` 段 |
| `config/controller_mppi.yaml` | MPPI 档：同上结构 |
| `rviz/nav2.rviz` | 由 nav2 默认视图裁剪（删 TB3 专属项，增 `/trajectories`），固定帧 map |
| `scripts/nav_waypoints.py` | `nav2_simple_commander` 航点回归脚本（客厅→门洞→厨房→门洞→卧室） |

`map_server.yaml_filename` 必须显式写在 yaml 里的原因：`localization_launch.py` 用 `RewrittenYaml`
按 `map:=` 重写该键，而它只替换已存在的键（源码已确认），漏写则静默用空路径启动失败。

## 10. 验证方案

见 §11 表格（无头 CLI 可自动完成 1–5、8；RViz 交互 2D Pose Estimate / 2D Goal Pose 由人确认）。

## 11. 验收记录

前置：`ros2 launch car_navigation navigation.launch.py gui:=false nav_rviz:=false`（无头）。

| # | 项目 | 指令 | 实测 | 结论 |
|---|---|---|---|---|
| 1 | 生命周期就绪 | `ros2 lifecycle get /amcl`、`/controller_server`、`/bt_navigator` | 待填 | 待填 |
| 2 | 地图与 TF 链 | `ros2 topic echo /map --once`、`ros2 run tf2_tools view_frames` | 待填 | 待填 |
| 3 | 初始定位 | `ros2 run tf2_ros tf2_echo map base_footprint`（起步时） | 待填 | 待填 |
| 4 | 代价地图 | `ros2 topic hz /global_costmap/costmap`，RViz 看膨胀不封死门洞 | 待填 | 待填 |
| 5 | 单点导航 | `ros2 action send_goal /navigate_to_pose ...`（目标 (4.0,2.0) 门洞） | 待填 | 待填 |
| 6 | 航点回归 | `ros2 run car_navigation nav_waypoints` | 待填 | 待填 |
| 7 | 控制器切换 | 同目标 `controller:=mppi` 重跑 | 待填 | 待填 |
| 8 | 停车兜底 | 导航中 `pkill -x controller_server`（**勿用 `pkill -f`**） | 待填 | 待填 |

## 12. 暂不做（留 TODO）

- TEB 档（源码编译 `teb_local_planner` + `costmap_converter` 后按 §6 接入）。
- Qt 面板下发导航目标 / 回传导航状态（`car_route` 信封协议扩展，另开设计文档）。
- 与通信层并存的仲裁（引入 `twist_mux` 或自写仲裁节点）。
- `collision_monitor`、多楼层、`waypoint_follower` 的高级任务执行器、实物联调（参数按仿真时钟固化）。
