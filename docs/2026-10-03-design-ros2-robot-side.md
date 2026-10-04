# wheels ROS2 端（机器人侧）接口设计文档

日期：2026-10-03
主题：机器人侧网关/转换双节点（C++）与底盘标准接口对接
状态：已实施（2026-10-03，落地于本仓库 `src/car_route`，包名 `car_route`，可执行名 `gateway` / `translator`；§3/§6/§9 的 Qt 侧契约未变）
相关文档：`2026-10-02-design-ros-transport.md`（Qt 侧 TCP 传输与 Python 桥接，本文档的前置）

## 1. 背景与目标

现状：Qt 面板 ros 通道已通——`RosTransport`（TCP+NDJSON）↔ `ros_bridge/wheels_bridge.py`（rclpy 透传）↔ `/wheels/uplink|downlink` 信封话题。机器人侧只有 Python 桥接脚本与 `fake_telemetry.py` 联调脚本，**没有真正消费 cmd_vel / 产生遥测的功能节点**。

目标：设计 ROS2 机器人侧节点，把 JSON 信封协议接到底盘驱动包的标准接口 `/cmd_vel`、`/odom` 上。

需求确认结论：

| 项 | 结论 |
|---|---|
| 架构模式 | **网关层隔离**：一个节点只做网络包抓取/透传，另一个节点在订阅回调里处理 JSON |
| 拓扑 | 保留"透传 + 转换"两节点结构 |
| 实现技术 | 机器人侧**全部 C++（rclcpp），避免 Python** |
| 与本项目关系 | **机器人侧零项目依赖**：只依赖 §3 的 JSON 包协议规格，不引用、不编译本项目任何源码 |
| 底盘对接 | 已有底盘驱动包，标准 `/cmd_vel`（geometry_msgs/Twist）+ `/odom`（nav_msgs/Odometry），新节点只做转换 |
| Qt 侧 | **零改动**：端点格式、信封、话题名、QoS、参数名全部不变 |

## 2. 总体架构（网关层隔离）

```
Qt 面板 RosTransport（ITransport，QTcpSocket，host:port 默认 10000）
   │  TCP NDJSON：每行一个完整 JSON 信封，'\n' 定界
   ▼
[网关节点 wheels_gateway]（C++/rclcpp）
   职责 = TCP 服务端 + 帧定界 + std_msgs/String 透传，无任何 JSON 语义
   │  /wheels/uplink  （String，Qt→ROS：cmd_vel / heartbeat）
   │  /wheels/downlink（String，ROS→Qt：telemetry / heartbeat）
   ▼
[转换节点 wheels_translator]（C++/rclcpp）
   职责 = 订阅回调里按 §3 协议规格做 JSON ↔ ROS 原生消息转换
   │  /cmd_vel（Twist）      → 底盘驱动包
   │  /odom  （Odometry）    ← 底盘驱动包
```

- 网关与转换节点之间仍是信封话题，因此 `fake_telemetry.py`、`ros2 topic pub/echo` 等工具可直接接入任一环节联调。
- 网关在机器人上**替代 `wheels_bridge.py` 的生产角色**；两个 Python 脚本暂留仓库作联调/对照（删除时机见 §12）。
- 转换节点**自带信封编解码**（按 §3 规格实现，仅需一个 JSON 库单头），对 wheels player 项目零感知——协议是双方唯一的耦合面。

## 3. JSON 包协议（自包含，机器人侧唯一契约）

传输：TCP 上 NDJSON 定界（每包一行 UTF-8 JSON + `'\n'`）；ROS 侧 `std_msgs/String` 透传，`String.data` 即一行原始 JSON。

### 3.1 信封（三种包共用）

```json
{"type":"<类型>","seq":<int64>,"ts_ms":<int64>,"data":{...}}
```

| 字段 | 类型 | 说明 |
|---|---|---|
| `type` | string | `"cmd_vel"` / `"telemetry"` / `"heartbeat"`，其他值整包作废 |
| `seq` | int64 | 发送方内自增序号 |
| `ts_ms` | int64 | Unix 纪元毫秒（发送时刻） |

### 3.2 cmd_vel（Qt → 机器人，按键按下/松开时发送）

```json
{"type":"cmd_vel","seq":42,"ts_ms":1696000000000,
 "data":{"linear":{"x":0.5,"y":0.0,"z":0.0},"angular":{"x":0.0,"y":0.0,"z":0.8}}}
```

- `linear.x/y/z` 线速度 m/s，`angular.x/y/z` 角速度 rad/s；六个 double **全部必填、无默认值**
- 松开按键 = 全零包；对应 `geometry_msgs/Twist` 同名字段直映射

### 3.3 telemetry（机器人 → Qt）

```json
{"type":"telemetry","seq":17,"ts_ms":1696000000100,
 "data":{
   "position":{"x":1.2,"y":0.3,"z":0.0},
   "orientation":{"roll":0.0,"pitch":0.0,"yaw":1.57},
   "linear":{"vx":0.5,"vy":0.0,"vz":0.0},
   "angular":{"wx":0.0,"wy":0.0,"wz":0.8}}}
```

- `position` 米；`orientation` 弧度（ZYX 欧拉角）；`linear.vx/vy/vz`、`angular.wx/wy/wz`；全部必填 double
- 对应 `nav_msgs/Odometry`：`pose.pose.position` 直映；`pose.pose.orientation` 四元数→欧拉；`twist.twist.linear/angular` 直映（见 §7）

### 3.4 heartbeat（双向）

```json
{"type":"heartbeat","seq":5,"ts_ms":1696000000200}
```

无 `data` 字段。

### 3.5 解析规则

JSON 语法错 / 缺 `type`、`seq`、`ts_ms` / cmd_vel、telemetry 缺 `data` 或 `data` 内任一子字段 / `type` 未知 → **整包丢弃**并计数，不抛异常、不部分生效。数值字段按 double 解读（整数形式也接受）。

## 4. 网关节点 wheels_gateway

行为规则自 `wheels_bridge.py` 平移（这是"Qt 侧零改动"的保证）：

- TCP 服务端监听 `0.0.0.0:port`（默认 10000）；**单客户端，新连接顶掉旧连接**（半开连接自愈）。
- 收：追加行缓冲 → 按 `'\n'` 切行；行缓冲上限 64 KiB，超限清空重同步；整行 UTF-8 解码 + JSON 合法性校验通过后，`String.data = 原始行` 发布 uplink。非法行丢弃 + 计数日志，**连接不断**。
- 发：订阅 downlink，每条 `String.data` 追加 `'\n'` 写回客户端；无客户端期间丢弃并计数；发送缓冲积压时监听可写事件。
- 线程模型：rclcpp executor 线程 + TCP 收发线程，线程安全队列交接（`std::mutex` + `std::queue`），避免跨线程调用 rclcpp。
- 参数名与默认值与 `wheels_bridge.py` 一致：`port` / `uplink_topic` / `downlink_topic`（见 §9）。

## 5. 转换节点 wheels_translator

上行（遥控指令）：

- 订阅 uplink → 按 §3.5 解析信封：
  - `cmd_vel` → 字段直映射（§7）发 `/cmd_vel`，并刷新 watchdog 时间戳；
  - `heartbeat` → 计数 + 节流日志（内容暂不驱动逻辑）；
  - `telemetry`/未知类型/解析失败 → 丢弃 + 计数日志。
- **cmd_vel watchdog**：超过 `cmd_vel_timeout_ms`（默认 500，0=关）未收到 cmd_vel → 每 100 ms 持续发布全零 Twist，收到新 cmd_vel 即停。Qt 断链时网关不再有 uplink 流量，watchdog 兜底停车；正常操控时 Qt 松开按键本就发全零，watchdog 平时不动。

下行（遥测上报）：

- 订阅 `/odom` 缓存最新一帧；定时器按 `telemetry_rate_hz`（默认 10，0=每条 odom 都转）节流 → 按 §3.3 组装信封（`seq` 节点内自增，`ts_ms` 取本机毫秒）→ String 发 downlink。
- heartbeat：定时器按 `heartbeat_rate_hz`（默认 1.0，0=关）发心跳信封（Qt 端已能接收打印，用于区分"ROS 活着但无 odom"与"链路断"）。

## 6. 接口契约总表

TCP 端点与 Qt 侧约定不变：`host:port`，NDJSON，单客户端。

| 话题 | 类型 | 发布方 → 订阅方 | QoS | 说明 |
|---|---|---|---|---|
| `/wheels/uplink` | std_msgs/String | 网关 → 转换 | reliable, depth 10 | 指令不可丢 |
| `/wheels/downlink` | std_msgs/String | 转换 → 网关 | best_effort, depth 10 | 遥测容忍偶发丢帧，与现桥接一致 |
| `/cmd_vel` | geometry_msgs/Twist | 转换 → 底盘驱动 | reliable, depth 1 | 只关心最新速度 |
| `/odom` | nav_msgs/Odometry | 底盘驱动 → 转换 | best_effort, depth 10 | 传感器惯例（REP 2003），对可靠/不可靠发布者均兼容 |

四个话题名均可经 ROS 参数覆盖（§9），但默认值构成对 Qt 侧的冻结契约：改默认值 = 破坏兼容，需同步修订 Qt 侧文档。

## 7. 数据映射

上行 cmd_vel 信封 → `geometry_msgs/Twist`（同名字段直映射，解析已保证六字段齐全）：

| 信封字段 | Twist 字段 |
|---|---|
| `data.linear.x / y / z` | `linear.x / y / z` |
| `data.angular.x / y / z` | `angular.x / y / z` |

下行 `nav_msgs/Odometry` → telemetry 信封：

| Odometry 字段 | 信封字段 |
|---|---|
| `pose.pose.position.x / y / z` | `data.position.x / y / z` |
| `pose.pose.orientation`（四元数） | `data.orientation.roll / pitch / yaw`（ZYX 欧拉角，用 tf2：`tf2::Matrix3x3::getRPY`） |
| `twist.twist.linear.x / y / z` | `data.linear.vx / vy / vz` |
| `twist.twist.angular.x / y / z` | `data.angular.wx / wy / wz` |

- `ts_ms` 取**编码发送时刻**本机毫秒（与 Qt 端及 `fake_telemetry.py` 语义一致）；不用 `header.stamp`，避免时钟源差异，Qt 端只显示不校时。
- `seq` 转换节点内 int64 自增。

## 8. 校验规则（接口校验，内部信任）

| 边界 | 规则 | 失败处置 |
|---|---|---|
| 网关收 TCP | UTF-8 解码 + JSON 合法、行长 ≤ 64 KiB | 丢行 + 计数日志，连接不断 |
| 转换收 uplink | 信封按 §3.5 解析成功且 type ∈ {cmd_vel, heartbeat} | telemetry / 未知 / 解析失败 → 丢弃 + 计数，不崩溃不抛异常 |
| 转换收 /odom | 形状由 ROS 类型系统保证；四元数全零（未初始化）时欧拉角记 0 | 记 0 + 一次性警告日志 |
| 转换发 /cmd_vel | watchdog 持续兜底 | — |

## 9. 参数表

| 节点 | 参数 | 默认值 | 说明 |
|---|---|---|---|
| wheels_gateway | `port` | 10000 | TCP 监听端口 |
| | `uplink_topic` | `/wheels/uplink` | |
| | `downlink_topic` | `/wheels/downlink` | |
| wheels_translator | `uplink_topic` | `/wheels/uplink` | 与网关对接 |
| | `downlink_topic` | `/wheels/downlink` | 与网关对接 |
| | `cmd_vel_topic` | `/cmd_vel` | |
| | `odom_topic` | `/odom` | |
| | `telemetry_rate_hz` | 10.0 | 0 = 每条 odom 都发 |
| | `cmd_vel_timeout_ms` | 500 | watchdog，0 = 关闭 |
| | `heartbeat_rate_hz` | 1.0 | 0 = 关闭 |

## 10. 包与构建

本仓库内 `src/car_route/`：**自包含**的 ament_cmake 包，两节点一包，与仓库内其他包无源码引用：

```
src/car_route/
  package.xml                    # 依赖 rclcpp、std_msgs、geometry_msgs、nav_msgs、tf2
  CMakeLists.txt
  third_party/nlohmann/json.hpp  # vendor 单头（v3.11.3，升级手工同步）
  src/gateway_node.cpp           # 可执行名 gateway（节点名 gateway）
  src/translator_node.cpp        # 可执行名 translator（节点名 translator）
```

- 信封编解码按 §3 规格在 `translator_node.cpp` 内自实现（JSON 库仅用 nlohmann 单头）；协议演进时双方以 §3 为准各自同步。
- `src/car_route` 随本仓库 colcon 工作空间一起构建，不依赖 wheels player 源码树的位置。
- 构建与运行（ROS2 Humble + colcon）：

```bash
cd ~/projects/ros_car
colcon build --packages-select car_route
source install/setup.bash
ros2 run car_route gateway
ros2 run car_route translator
```

## 11. 验证方案

无实车（ROS 机器或本机 WSL/虚拟机装 ROS2 Humble）：

1. 起 C++ 网关；Qt 顶栏选 ros、填 `host:10000` → 连接变绿（网关替代 `.py` 后上行链路等价）。
2. `fake_telemetry.py`（发 `/wheels/downlink` 信封）→ Qt 面板滚动（下行链路等价）。
3. Qt 按箭头 → `ros2 topic echo /cmd_vel` 出值、松开回全零。
4. `ros2 topic pub -r 10 /odom nav_msgs/msg/Odometry ...` 喂固定位姿 → Qt 面板数值随之。
5. watchdog：Qt 发非零 cmd_vel 后直接杀掉 Qt → `/cmd_vel` 500 ms 内回到全零。
6. 异常输入：`ros2 topic pub --once /wheels/uplink std_msgs/msg/String "{data: 'not json'}"` → 两节点均不崩、计数日志递增。

有实车：起底盘驱动包后重复 3、4，Qt 全程操控；观察断链（关 Qt）后机器人停止。

## 12. 暂不做（留 TODO）

- 网关多客户端并发（单客户端满足一控一机）。
- 上行 heartbeat 驱动 Qt 链路状态显示（Qt 侧仅 qDebug、转换节点仅计数）。
- Qt 侧周期心跳上报（沿袭 `2026-10-02-design-ros-transport.md` 的 TODO）。
- nav/TF、地图、任务级集成。
- Python 脚本删除时机：`wheels_bridge.py` 待 C++ 网关联调稳定后由用户决定；`fake_telemetry.py` 在无车联调中仍有用，建议保留。
