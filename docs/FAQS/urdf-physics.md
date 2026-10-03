# URDF / 物理仿真类

## STL 网格直接加载位置全错
- **现象**：URDF 里引用 STL 后模型歪七扭八。
- **原因**：STL 无单位（本项目为**毫米**，需 scale 0.001），且所有部件在同一装配坐标系（本项目约定：**-Y 是车头、地面在 z=-52.8mm**），网格顶点是绝对坐标。
- **解决**：每个 mesh origin 加 `rpy="0 0 π/2"`（装配系→ROS 系），joint 放位置、visual origin 放反向偏移把网格"推回"装配位置。

## 车身稳定停在 ~1.3° 倾斜、雷达扫到地面
- **现象**：静止后 roll/pitch 恒定不为零，/scan 后向出现 7~12m 的地面回波。
- **原因**：base_footprint 上加了个装饰性小球碰撞 → 原点处多了第 4 个支撑点，车身能"稳定"停在单轮悬空的姿态（右轮悬 2.4mm）。
- **解决**：base_footprint 只留 visual，**严禁加碰撞**。

## 车头上翘（配平问题）
- **现象**：pitch 恒 +1°，前万向轮悬空。
- **原因**：按网格位置放质量，电池/电机/主控都在驱动轴之后，整车 COM 落在驱动轴**后方** 38mm——三轮车要求 COM 必须在驱动轴之前、前后支撑点之间。
- **解决**：底盘/电池的 inertial origin 前移，把 COM 移到驱动轴前 61mm。

## 万向轮卡死/转向困难
- **解决**：万向轮用固定球碰撞 + `mu=0`（近零摩擦）模拟，这是三轮差速车的标准做法。

## 静止时缓慢爬行 / 缓慢自转 / IMU 某轴有恒定角速度
- **现象**：无指令时车以 mm/s 级爬行、缓慢棘转；IMU ω_y 恒 ~0.2 rad/s 但姿态/雷达/加速度都显示静止。
- **原因**：diff_drive 插件零速时把轮子"锁死"（速度伺服），与接触参数（尤其大 kd）耦合出高频微振极限环：振幅极小（姿态看不出来）但瞬时角速率大，摩擦各向异性导致净漂移。
- **解决**：轮子接触 kp/kd **不要手动设置**（用默认），关节 damping 保持小值。残余 ω_y ~0.06 rad/s 伪影无害：EKF/融合对地面机器人只采 IMU 的 z 轴 yaw 率（姿态和加速度通道是准的）。

## 左右轮装反（转向方向相反）
- **现象**：+angular.z 指令车却反着转。
- **原因**：转向符号由「关节 y 坐标 + 插件 <left_joint>/<right_joint> 指定」决定，与网格文件内容无关。
- **解决**：检查插件 left/right 是否真的对应 +y(左)/−y(右) 侧关节。**新建差速模型务必先做转向符号测试**。

## 左右轮网格配对装反（视觉在另一侧，物理无异常）
- **教训**：网格配对只能用旋转矩阵推导——本模型映射为 x_ros=+x_m，故 x_m≥0 的半块渲染在 +y=左侧。曾因一次被接触抖动污染的转向实测去翻转网格配对，结果视觉左右互换而物理毫无变化（转向测试根本验证不了网格配对）。
- **验证方法**：拆分后读网格包围盒符号即可程序化判定渲染侧，不要靠跑仿真。

## 空轮关节 + 固定视觉轮的 URDF 无法仿真
- **坑**：视觉轮放在 fixed link、left/right 只是空 frame（无视觉/碰撞/惯量）——Gazebo 里转的是看不见的零质量 frame，可见轮子不动；关节若按网格外缘（±81mm）而非轮心（±64mm）设置，差速轮距错 26%。
- **解决**：轮关节 link 必须自带视觉+碰撞+惯量，位于轮心。

## 传感器 frame 不要放在网格包围盒中心
- **坑**：lidar 的 frame 放在网格几何中心（z_m 73mm），与真实扫描面（z_m 90mm）差 17mm；ray 传感器从 frame 原点出射，读数系统性偏差。
- **解决**：传感器 frame 放物理有效点（扫描面/声呐面/IMU 敏感中心），视觉网格用 visual origin 对齐到 frame。

## base_footprint 带惯量触发 KDL 警告
- **现象**：robot_state_publisher 报 `The root link base_footprint has an inertia specified in the URDF, but KDL does not support...`（无害但烦人）。
- **解决**：base_footprint 保持空 link `<link name="base_footprint"/>`——惯量放 base_link，碰撞本来就不该有。

## 雷达/超声波插件话题不出 / 报 Invalid output_type
- **现象**：`Invalid <output_type> [sensor_msgs/msg/LaserScan]`；超声波找不到 range 插件。
- **原因**：Humble 的 ray 传感器类型名**不带 msg/**；且 ROS2 Humble 没有 `libgazebo_ros_range.so`。
- **解决**：雷达用 `sensor_msgs/LaserScan`；超声波用 ray 传感器窄视场（5 束）+ `<output_type>sensor_msgs/Range`。

## IMU 噪声配置不生效/数据怪
- **原因**：Gazebo Classic 的 IMU 只支持**单一 `<imu><noise>` 块**，逐轴 `<x><noise>` 写法会被错误解析。
- **解决**：用单一 noise 块（type/mean/stddev/bias）。

## diff_drive 的 odometry TF 里混入俯仰横滚
- **原因**：`<odometry_source>` 默认 1 = 仿真世界真值。
- **解决**：设 `<odometry_source>0`（编码器积分），与实物 ESP32 轮式里程计行为一致。

## 车停不下来 / 测出的位移比指令大
- **原因**：diff_drive 插件**不读 cmd_vel_timeout**，指令流停止后目标速度一直保持。
- **解决**：上层（teleop/导航/测试脚本）必须持续发布 cmd_vel，停车要显式发 0。

## spawn 位置贴地导致奇怪姿态
- **解决**：spawn 时抬高 `-z 0.01` 让车自然落地（不要 3 个接触点零间隙同时接触）。
