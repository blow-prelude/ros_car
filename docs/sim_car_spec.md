# 仿真小车规格描述（xuegecar）

> 目标：在 ROS 2 Humble + Gazebo Classic 11 中验证 移动 / 定位 / 避障。
> 参考实物：src/car_description/demo.jpg（黑色多层板、后两轮差速驱动、前万向轮、上层 2D 雷达、车身 IMU、前部超声波+摄像头、底盘 ESP32-S3+电池+TT 电机）。
> 本文档尺寸均为从 meshes/*.stl 实测（STL 单位为毫米）。
> v2（2026-10-02）：按用户提供的 xuegecar URDF 描述重建——base_link 改为底盘包围盒中心约定，link 命名对齐（laser_frame/ultrasonic_link 等），补齐仿真层。

## 1. 总体形态

- 差速驱动三轮车：后两轮 TT 直流减速电机驱动，前轮万向球轮。
- 两层结构：下层底盘（电机、电池、ESP32-S3、IMU），上层板放 2D 雷达；超声波在前缘中部朝前，摄像头在前上方。
- 整体外廓：长 158mm（含轮悬出）× 宽 162mm（含轮）× 高 155mm（雷达顶），总质量 ≈ 2.1 kg（底盘惯量块 2.0 + 轮/万向轮）。

## 2. STL 网格坐标约定（重要）

所有 STL 共用同一网格坐标系（"装配系"），单位毫米：**−Y 是车头、+X 渲染到小车左侧、地面在 z=−52.8mm**。
网格系→ROS 映射（rpy 0 0 +π/2）：**x_ros=−y_m, y_ros=+x_m, z_ros=z_m** —— 部件左右/前后归属一律由此推导（x_m≥0 的半块在 +y=左），不要靠目测或单次实测猜。

**base_link 约定（v2，采纳 xuegecar 描述）**：base_link = 底盘包围盒中心，离地 0.06205m；所有网格共用摆放恒等式 **visual_origin = T − joint_origin**，其中 T=(0.00225, 0, −0.00925)，由 xacro 宏自动计算。轮心离地 0.0325=轮半径 ✓，万向轮球心离地 0.0109=球半径 ✓（自洽校验）。

meshes 文件：chassis / drive_wheels（按 x_m 拆分为 wheel_left[x_m≥0] / wheel_right[x_m<0]）/ caster_wheel / lidar / ultrasonic / camera / battery / controller / tt_motor。
**meshes/decimated/ 为减面副本（整车 61 万 → 10 万三角面），URDF 视觉默认引用它**——软件渲染下 gzclient 用高模会卡死；碰撞体全部为几何原语不受影响。

## 3. 机械结构（v2，坐标相对 base_link，米）

| 部件 | 关节/安装位姿 | 质量/惯量 | 碰撞 |
|---|---|---|---|
| base_footprint | 空 link（KDL 根链接限制 + 防多余接触点） | — | 无 |
| base_link | 原点=底盘包围盒中心，离地 0.06205 | 2.0 @ (0,0,0.02)，ixx 0.01 / iyy 0.02 / izz 0.02 | 主盒 0.1095×0.09×0.0805 @ (−0.0175,0,0) + 前段 0.035×0.09×0.0165 @ (0.05475,0,0.032) |
| chassis_link（视觉） | (0,0,0) | — | — |
| wheel_left/right_link | (−0.05075, **±0.064**, −0.02955)，continuous 绕 y | 各 0.03 | 圆柱 r=0.0325 厚 0.034 |
| caster_link | (0.0652, 0, −0.05115)，fixed | 0.02 | 球 r=0.0109，**mu=0** |
| laser_frame | (−0.0227, 0.000062, **0.08075**)=真实扫描面，离地 0.1428 | — | — |
| imu_link | (0, 0, 0.03) | 1g | — |
| ultrasonic_link | (0.042651, 0, 0.01225)，离地 0.0743 | — | — |
| camera_link | (0.047716, −0.000603, 0.052408) | — | — |
| battery_link | (−0.02475, 0, 0.00525) | — | — |
| controller_link | (−0.02475, −0.000108, 0.018005) | — | — |
| tt_motor_link | (−0.026904, 0, −0.02955) | — | — |

说明：
- 纯视觉 link 无碰撞/惯量（fixed 会被 Gazebo 并入 base_link）；碰撞/惯量集中在 base_link、轮、万向轮。
- 碰撞盒收窄到 0.09 避让驱动轮（内缘 y=0.047），前段抬高到声呐波束（顶 z=0.02275）之上——否则轮子接触抖动、超声波自遮挡。
- COM 在底盘包围盒中心 = 驱动轴前 50.7mm、万向轮后 65.2mm，满足三轮车稳定条件。
- xuegecar 原描述的轮关节 y=±0.081 是轮**外缘**（网格实测轮心 ±0.064），照抄会导致差速轮距错 26%。

## 4. 运动学与驱动

- 轮距 0.128m，轮半径 0.0325m，标准差速运动学。
- 速度限制（供上层使用）：v ≤ 0.35 m/s，ω ≤ 2.0 rad/s。
- 驱动：`gazebo_ros_diff_drive`（仿真中扮演 ESP32-S3+电机驱动+编码器）：
  - update_rate 50，max_wheel_torque 1.0，max_wheel_acceleration 3.0；
  - **`<odometry_source>0`（编码器积分）**——默认值 1 是世界真值，会把车体俯仰/横滚混进 odom TF；
  - **此插件不读 `cmd_vel_timeout`**：指令流停止后目标速度保持，上层必须持续发布 cmd_vel、停车发 0；
  - 上实物时 ESP32-S3 用 micro-ROS 提供相同 /cmd_vel、/odom 接口。

## 5. 传感器规格（已验证话题）

| 传感器 | 话题 | 参数 |
|---|---|---|
| 2D 雷达 | /scan (LaserScan) @ laser_frame | 10Hz，360 线，±π，0.10–12m，σ=0.01 |
| IMU | /imu/data @ imu_link | 200Hz；**只支持单一 noise 块**，逐轴 noise 写法会解析异常 |
| 超声波 | /ultrasound (Range) @ ultrasonic_link | 10Hz，5 射线模拟 15° 视场，0.02–4m；**Humble 无 libgazebo_ros_range.so，用 ray_sensor 的 `<output_type>sensor_msgs/Range`**（注意类型名不带 `msg/`） |
| 摄像头 | /camera/front/image_raw | 默认关闭（`with_camera:=true` 开启）；WSL 下 GPU 离屏渲染会导致 gzserver 崩溃，软件渲染时可用 |
| 轮式里程计 | /odom + TF odom→base_footprint | 编码器积分，含协方差；TF 发布者可选：默认 EKF 融合 IMU 发布，裸仿真传 `with_odom_tf:=true` 改由 diff_drive 插件发布（与 EKF 路径互斥，gazebo.launch.py 防呆强制） |

TF 树：odom → base_footprint → base_link → {wheel_left/right_link, caster_link, laser_frame, imu_link, ultrasonic_link, camera_link, chassis_link, battery_link, controller_link, tt_motor_link}

## 6. 已知问题与经验（踩坑记录）

1. **WSL2 Intel D3D12 驱动会让 gzserver 崩溃**（连空世界都加载不了）：必须 `export LIBGL_ALWAYS_SOFTWARE=1` 软件渲染。
2. **gzserver 端口 11345 释放慢**：kill 后要等几秒再启新实例，否则 "Address already in use"。
3. **同一世界 spawn 两辆车会因插件节点重名（/imu_plugin）崩溃**：每个测试变体用全新 gzserver。
4. **base_footprint 不能有碰撞几何**：原点处的接触点会成为第 4 个支撑点，车身会稳定停在 ~1.3° 倾斜姿态（单轮悬空 2.4mm），雷达因此扫到地面。
5. **接触 kp/kd 不要手动设置**：默认值最稳；自调参数（尤其大 kd）会与差速插件的速度伺服耦合出锁轮极限环（静止爬行、IMU ω_y 伪速率 0.06–0.25 rad/s）。
6. **IMU ω_y 存在 ~0.06 rad/s 常量伪速率**（该 gazebo 构建+铰接体+速度伺服的极限环残余，振幅极小不影响姿态/雷达/加速度）：EKF 配置只采 IMU 的 z 轴 yaw/yaw-rate（地面机器人标准做法）即可完全规避。
7. spawn 建议 `-z 0.01` 让车自然落地；**轮网格左右配对由旋转矩阵推导**（x_ros=+x_m ⇒ x_m≥0 半块在 +y 左侧），转向方向由关节 y 坐标+插件 left/right 指定决定，与网格无关。
8. 修改 URDF 后务必 `colcon build`——`$(find pkg)` 解析到 install/ 里的旧拷贝，不重建等于没改。
9. v2 模型 2.0kg 底盘：原地旋转里程计偏大 ~15%（轮子打滑，编码器积分特性），直线与停车行为与 v1 一致。**EKF 并非"融合了 IMU 就能校正"**（2026-10-15 实测纠正）：旧配置同时融合轮速绝对 yaw 时，`covariance_yaw`(0.001) 与 IMU `orientation_covariance`(0.01) 的权重比把 ~91% 信任给了更不准的轮速，导航累计旋转 ~64 rad 后 odom 偏航误差仍涨到 1.5 rad；必须让轮速只融合 vx/vyaw、航向交给 IMU（见 `car_localization/config/ekf.yaml` 与 `FAQS/nav-localization-drift.md`）。

## 7. 最终验收结果（空场，无头 gzserver）

| 项目 | 指令 | 实测 | 结论 |
|---|---|---|---|
| 前进 | 0.15 m/s × ~3s | +0.468m，航向偏差 -0.0° | ✓ |
| 原地转向 | +0.4 rad/s × ~3s | +72.3°（期望 ~69°） | ✓ 方向/幅度正确 |
| 怠速漂移 | 无指令 10s | -4.4mm（0.44mm/s） | ✓ |
| 停车后漂移 | 发 0 速后 5s | 1.8mm | ✓ |
| 姿态 | 静止 | roll/pitch ≈ 0° | ✓ |
| 雷达 | 空场 | 360 线全 inf（无地面回波） | ✓ |
| 超声波 | 空场 | 无自遮挡 | ✓ |

## 8. car_bringup（已创建）

```
src/car_bringup/worlds/home_world.sdf   # 家庭场景: 8x6m 两室一厅(客厅/卧室/厨房), 全静态家具, 雷达可见, 门洞 0.8m
src/car_bringup/launch/gazebo.launch.py # gzserver/gzclient + RSP + spawn(-3,-2 朝+x) + 可选 rviz
src/car_bringup/config/car.rviz         # Grid/RobotModel/TF/LaserScan/Odometry, 俯视 2D
```

用法：`ros2 launch car_bringup gazebo.launch.py [gui:=false|rviz:=true|with_camera:=true|spawn_x/y/yaw:=...]`（launch 内已设 LIBGL_ALWAYS_SOFTWARE=1）。

已验证：spawn 成功；雷达四向墙距与布局核算一致（前 3.36m 电视柜 / 后 1.09m 西墙 / 左 4.96m 北墙 / 右 0.94m 南墙）；odom→laser TF 链完整（-0.042, 0, 0.143）；/tf_static 正常。

勘误（2026-10-07）：本节原先记载"/joint_states 正常"失实——全仓库当时无任何发布者，RSP 收不到连续关节（轮）状态。现由 car.gazebo.xacro 的 libgazebo_ros_joint_state_publisher.so 以 50Hz 发布轮角，同时关闭 diff_drive 的 publish_wheel_tf（避免与 RSP 双发布轮子 TF）。

## 9. 后续（未创建）

```
src/car_bringup/launch/teleop.launch.py   # teleop 必须持续发布 cmd_vel
src/car_bringup/launch/slam.launch.py     # slam_toolbox 建图
src/car_bringup/launch/nav.launch.py      # map_server + AMCL + nav2
src/car_bringup/config/{ekf.yaml, nav2.yaml}
```
