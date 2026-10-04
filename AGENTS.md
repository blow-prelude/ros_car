# Repository Guidelines

ROS 2 Humble + Gazebo Classic 11 仿真小车工作区（WSL2）。差速三轮车 `xuegecar`：移动 / 定位 / SLAM / 外部 Qt 面板遥控遥测。**注释与文档一律中文**，写清设计动机与踩坑原因。

## 项目概述

* 目标：在仿真中验证差速车的 移动 → 定位 → 建图 链路，并通过 TCP/JSON 协议对接外部 Qt 遥控面板（`/cmd_vel`、`/odom` 标准底盘接口）。

* 5 个 `ament_cmake` 包，构建系统 colcon；无 Python 功能包（机器人侧节点全 C++/rclcpp）。

* 改 URDF 前必读 `docs/sim_car_spec.md`；遇 Gazebo/launch 异常先查 `docs/FAQS/`。

## 架构与数据流

四层结构，每层单一职责，层间只通过标准化接口（topic + QoS、参数、JSON 信封协议）通信：

```
仿真层   car_description(模型+Gazebo插件) + car_bringup(世界+总启动launch)
感知定位层 car_localization(imu_filter 互补滤波 + robot_localization EKF)
建图层   car_slam(slam_toolbox, 纯配置包)
通信层   car_route(gateway TCP/NDJSON 透传 + translator JSON↔ROS 转换)
```

数据流（topic 链路）：

```
Gazebo 插件(urdf/car.gazebo.xacro)
  ├─ /imu/data (Imu,200Hz) → imu_filter → /imu/data_filtered ─┐
  ├─ /odom (Odometry,编码器积分) ─────────────────────────────┤→ ekf_filter_node → /odometry/filtered
  └─ /scan (LaserScan,10Hz) ────────────────────────────────→ slam_toolbox → /map
                                                              + TF odom→base_footprint (EKF)
                                                                + TF map→odom (slam_toolbox)
/odom → translator → /wheels/downlink(String, best_effort) → gateway → TCP:10000 → Qt 面板
TCP:10000 → gateway → /wheels/uplink(String, reliable) → translator → /cmd_vel(Twist, reliable d1) → diff_drive
```

* **TF 唯一发布者**：`odom→base_footprint` 只由 EKF 发布（diff\_drive 插件 `publish_odom_tf: false`）；`map→odom` 由 slam\_toolbox。新增节点不得重复发布。

* **QoS 契约（冻结）**：`/cmd_vel` reliable depth1；`/odom`、`/wheels/downlink` best\_effort depth10（REP 2003）；`/wheels/uplink` reliable depth10；IMU 订阅用 `SensorDataQoS()`。改默认值 = 破坏外部兼容。

* **car\_route JSON 信封协议**是机器人与 Qt 侧的唯一耦合面（`docs/2026-10-03-design-ros2-robot-side.md` §3）：NDJSON、信封 `{type,seq,ts_ms,data}`、解析失败整包丢弃+计数、不抛异常。

### 架构设计原则（新增代码必须遵守）

1. **分层架构**：每层只负责一类功能；层间只经标准接口通信——不得跨层直接引用内部实现（例：launch 层不 import 他包 Python；节点间不共享头文件，只走 topic）。
2. **模块解耦**：包之间依赖接口而非实现。范例：`car_route` 对 Qt 侧零源码依赖（协议即接口）；`car_slam` 只 `IncludeLaunchDescription` 复用 `car_bringup/gazebo.launch.py`（launch args 即接口）；`gazebo.launch.py` 经 `localization.launch.py` 接口复用定位栈。
3. **文件原子化**：一个文件只做一件事——每节点一文件（`<node>_node.cpp`）、每场景一 launch、每功能一 config yaml。新文件职能超过一类时拆分。

## 关键目录

```
src/
  car_description/   纯数据包：urdf/car.urdf.xacro(本体) car.gazebo.xacro(全部Gazebo插件)
                     meshes/decimated/(减面网格,URDF默认引用,勿改回高模)
  car_bringup/       纯数据包：launch/gazebo.launch.py(仿真总入口) worlds/home_world.sdf config/car.rviz
  car_localization/  C++：src/imu_filter_node.cpp  config/ekf.yaml  launch/localization.launch.py
  car_slam/          纯数据包：config/slam_toolbox.yaml  launch/slam.launch.py  rviz/slam.rviz
  car_route/         C++：src/gateway_node.cpp translator_node.cpp  third_party/nlohmann/json.hpp(vendor单头)
docs/                中文文档：sim_car_spec.md(车体规格/坐标约定/踩坑) FAQS/(现象-原因-解决四段式)
                     2026-10-03-design-ros2-robot-side.md(car_route协议设计)
docs/devel/          新设计文档存放处（见「工作规范」）
.worktree/           大改动的 git worktree 存放处
build/ install/ log/ colcon 产物（git 忽略，勿手改 install/）
```

## 开发命令

```bash
# 每个 shell 先 source（bash 用 setup.bash）
source /opt/ros/humble/setup.bash
cd ~/projects/ros_car

# 构建：只编改动的包；改 URDF/xacro 后必须重建（xacro 解析的是 install/ 旧拷贝）
colcon build --symlink-install --packages-select car_localization
source install/setup.bash

# 运行（Gazebo 同一时间只能一个实例，kill 后等 3~5s 再启）
ros2 launch car_bringup gazebo.launch.py                 # 总入口，默认含定位栈；gui:=false 无头
ros2 launch car_slam slam.launch.py gui:=false           # 建图（内部强制 rviz:=false，RViz 用 slam_rviz:=true）
ros2 run car_route gateway                               # TCP:10000 ↔ /wheels/uplink|downlink
ros2 run car_route translator                            # /wheels/* ↔ /cmd_vel|/odom

# 测试（现仅 lint，见「测试与质量保障」）
colcon test && colcon test-result --all
```

## 代码规范与常见模式

**命名**（沿用现有代码）：类 `PascalCase` + `Node` 后缀（`ImuFilterNode`）；成员变量尾下划线（`alpha_`）；自由函数 snake\_case、常量 `k` 前缀（`kGravity`）；节点名字符串小写（`"imu_filter"`）；文件 `<节点>_node.cpp`。

**既有模式**（新代码照抄，勿发明第二套）：

* 参数：构造函数内 `declare_parameter<T>("名", 默认值)` 返回值绑 `const auto`。

* 回调：lambda `[this]` 内联委托私有成员方法（`on_imu`/`on_odom`）；定时器 `create_wall_timer`。

* 日志：`RCLCPP_INFO/WARN/ERROR(get_logger(), ...)`；重复刷屏用 `_THROTTLE(..., 5000, ...)` 或 `_ONCE`；异常输入计数器（`parse_failures_`）+ 限流 WARN，不抛异常。

* 错误恢复：自愈式——新 TCP 连接顶掉旧连接、64KiB 行缓冲超限重同步、发送积压踢慢客户端。

* 线程：非 executor 线程只碰 mutex 保护的队列，rclcpp 调用全在 executor 线程；跨线程计数 `std::atomic`。

* launch 三段式：`DeclareLaunchArgument`（默认值+用法注释写文件头 docstring）→ Node/Include 列表 → `return LaunchDescription([...])`；robot\_description 必须用 `ParameterValue(Command(['xacro ', ...]), value_type=str)` 包裹。

**编码要求**：

* 日志输出要充分，debug 级别稍多一些：正常路径用 `RCLCPP_DEBUG` 标注分支走向与中间量（状态机切换、滤波中间值、丢包原因），`INFO` 只留关键事件，异常分支 `WARN/ERROR`。

* 接口做类型检验与数据校验，**内部实现信任接口已做的校验，不做重复防御**：校验集中在边界一处（节点入口解析函数如 `parse_envelope`/`extract_twist`、launch 参数声明、QoS 选型、TCP 帧定界）；通过边界后的内部函数不再判空/判类型/判范围。校验失败处置=整包丢弃+计数+限流日志，不抛异常、不部分生效。

* 代码保持可读性与简约性：无第二实现前不引入抽象基类/DI/接口层；删净无用代码与注释掉的死代码；注释解释"为什么"（踩坑/约束），不复述"做了什么"。

## 重要文件

| 文件                                            | 说明                                                            |
| --------------------------------------------- | ------------------------------------------------------------- |
| `src/car_bringup/launch/gazebo.launch.py`     | 仿真总入口：Gazebo + RSP + spawn + 条件 include 定位栈                   |
| `src/car_description/urdf/car.urdf.xacro`     | 本体模型（坐标约定与惯量/碰撞设计见 `docs/sim_car_spec.md`）                    |
| `src/car_description/urdf/car.gazebo.xacro`   | 全部 Gazebo 插件：diff\_drive、雷达、IMU、超声波、可选相机                      |
| `src/car_localization/config/ekf.yaml`        | EKF 融合：odom0=/odom + imu0=/imu/data\_filtered，two\_d\_mode    |
| `src/car_slam/config/slam_toolbox.yaml`       | 建图参数：mode=mapping，base\_frame=base\_footprint                 |
| `src/car_route/third_party/nlohmann/json.hpp` | vendor 单头（v3.11.3），勿替换为系统依赖                                   |
| `docs/sim_car_spec.md`                        | 车体规格、STL 毫米装配系约定、9 条踩坑记录                                      |
| `docs/FAQS/*.md`                              | 实测验证过的 FAQ：`gazebo-wsl.md`、`ros2-launch.md`、`urdf-physics.md` |
| `docs/2026-10-03-design-ros2-robot-side.md`   | car\_route 设计（§3 JSON 协议=冻结契约，§11 验证方案）                       |

## 运行时与工具链

* **固定栈**：ROS 2 Humble（`/opt/ros/humble`）+ Gazebo Classic 11 + `gazebo_ros` 插件。**不是** `ros_gz`/gz sim——勿引入新版插件名或 `libgazebo_ros_*_so` 之外的等价物。

* C++17，`-Wall -Wextra -Wpedantic`；colcon isolated 布局；zsh/bash 均可用。

* **软件渲染是硬约束**：WSL2 下 `LIBGL_ALWAYS_SOFTWARE=1` 已写入用户 shell 环境变量（launch 文件未代设，勿在代码中覆盖或移除该依赖，**禁止改用硬件渲染**）。连带约束：URDF 视觉必须用 `meshes/decimated/` 减面网格；世界 `shadows:false`；摄像头默认关（`with_camera:=false`，GPU 离屏渲染会崩 gzserver）。

* 已知环境坑（详见 `docs/FAQS/gazebo-wsl.md`）：单 Gazebo 实例（端口 11345 释放慢）；杀进程用 `pkill -x` 勿用 `pkill -f`（会杀终端）。

## 工作规范（AI 协作约定）

* **需求不清时询问用户，不猜测**：澄清到可执行为止再动手；不要基于臆测的补充需求扩大范围。

* **设计文档**：用户开启 plan 模式或明确要求时，设计文档写入 `docs/devel/YYYY-MM-DD-design-<主题>.md`；改动较复杂时另写实现文档 `YYYY-MM-DD-plan-<主题>.md`，同一路径。历史设计文档在 `docs/` 根（旧惯例），新文档一律进 `docs/devel/`。

* **大变动用 worktree**：较大改动时新建 git worktree 到根目录 `.worktree/` 下（如 `.worktree/<主题>/`），验证由用户决定是否合并回主工作区；小改动直接在主工作区。

* git：提交信息中文短句（范例 `v0.1 仿真世界可以启动`），不强制 conventional commits。

## 测试与质量保障

* **现状**：全工作区无功能测试。`colcon test` 仅在 `car_route` 跑 `ament_lint_common` 静态检查（`cpplint`、`copyright` 显式禁用）；其余包为 no-op。无 clang-format/ruff 等。

* **验收惯例**：仿真场景手动验证——按设计文档 §11 风格给出 指令 → 实测 → 结论；改传感器/TF 后用 `ros2 topic hz/echo`、`tf2_tools view_frames` 核对链路。

* **新增测试**：放 `src/<pkg>/test/`，CMakeLists `BUILD_TESTING` 块 + `ament_add_gtest`；launch 级集成用 `launch_testing`。回归场景优先级：translator watchdog 500ms 兜底零速；非法 JSON 不崩且计数递增；EKF 是 odom→base\_footprint 唯一发布者；改 URDF 后雷达扫不到地面（base\_footprint 误加碰撞的老坑）。

