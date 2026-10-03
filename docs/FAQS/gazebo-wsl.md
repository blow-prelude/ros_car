# Gazebo @ WSL2 环境类

## gzclient 视口全黑（GUI 菜单正常，3D 区域黑屏）
- **现象**：Gazebo 窗口菜单/面板正常，但 3D 视口全黑；无报错。其他仿真正常。
- **原因**：WSLg 下 gzclient（Qt 程序）默认选 **Wayland 后端**，而软件渲染（LIBGL_ALWAYS_SOFTWARE=1 的 llvmpipe）在 EGL-Wayland 路径下输出黑屏；X11/GLX 路径正常（可用 `glxgears` 验证）。
- **解决**：launch 里加 `QT_QPA_PLATFORM=xcb` 强制走 X11(XWayland)（已加入 gazebo.launch.py，与软件渲染叠加使用）。
- **调试提示**：多窗口环境下别按"最大窗口"截图定位——用 `_NET_WM_PID` 属性把窗口和进程 PID 精确配对，否则会抓到别的程序的窗口得出假结论（本项目实测踩过：把 RViz 窗口当成 Gazebo 分析了一圈）。

## 两个 Gazebo 实例同时运行 → 加载页卡死/世界丢失/行为诡异
- **现象**：gzclient 卡在 "Preparing your world..."、场景缺模型、第二个实例的 gzserver 抢不到 11345 端口变成"无主"运行（物理和 ROS 插件正常、GUI 连不上）。
- **原因**：11345 主端口只能归一个实例；两个实例的 gztransport 互相干扰。
- **解决**：同一时间只跑一个 Gazebo 实例；确需并行时给每个实例设不同的 `GAZEBO_MASTER_URI`（如 http://localhost:11346）。GUI 现象诡异时先检查 `pgrep -x gzserver` 是不是不止一个。

## gzclient 打开后完全卡死（视角拖不动，无报错）
- **现象**：`ros2 launch car_bringup gazebo.launch.py` 一切正常加载完，但 Gazebo 窗口冻住、相机拖不动；无头模式（gui:=false）却一切正常。
- **原因**：软件渲染（LIBGL_ALWAYS_SOFTWARE=1）下 gzclient 每帧用 CPU 光栅化整车 STL 高模（controller 30 万面 + 电机 22 万面，整车 61 万三角），再加阴影多 pass 渲染，llvmpipe 直接被打满。gzserver 无头时不画网格所以没感觉。
- **解决**：①meshes/decimated/ 减面网格（整车 61 万 → 10 万面，URDF 已默认指向），原 CAD 高模保留在 meshes/ 顶层；②世界文件关阴影（`<scene><shadows>false</shadows></scene>`）。改完后 gzclient CPU 从满载降到 ~10%，RTF 0.93。
- **工具**：`pip3 install --user "trimesh==4.1.6" fast-simplification`（新版 trimesh 需 numpy≥2，Ubuntu 22.04 的 numpy 1.x 只能用旧版）；注意 `fast_simplification.simplify(v, f, r)` 的第三参是**要删除的面占比**（0.95 = 删 95%），不是保留占比。

## gzclient 启动后窗口白屏/无响应（30~45 秒）
- **原因**：软件渲染首次初始化 + 场景加载慢，属正常现象。
- **解决**：等 `/performance_metrics` 话题出现（或 CPU 从满载回落）再操作视图。

## gzserver 启动即崩溃 / 空世界都加载不了
- **现象**：日志出现 `D3D12: Removing Device` 或直接 segfault，连 empty.world 都起不来。
- **原因**：WSL2 的 Intel D3D12 显卡驱动让 OGRE 离屏渲染崩溃。
- **解决**：`export LIBGL_ALWAYS_SOFTWARE=1` 强制软件渲染（ray/imu 传感器完全够用，摄像头也能工作只是慢）。本项目已写进 gazebo.launch.py。

## 摄像头传感器导致整个仿真崩溃
- **现象**：带 camera sensor 的模型一 spawn，gzserver 就死。
- **原因**：同上，GPU 离屏渲染；软件渲染下正常。
- **解决**：摄像头做成 xacro 开关（`with_camera:=true` 才启用），默认关闭。

## 新 gzserver 报 "Address already in use"
- **现象**：kill 旧 gzserver 后立刻重启，报端口 11345 被占。
- **原因**：端口释放有延迟。
- **解决**：kill 后等 3~5 秒，`ss -tln | grep 11345` 确认释放再启动。

## 同一世界 spawn 两辆车 → gazebo 崩溃
- **现象**：第二辆 spawn 后报 `Found multiple nodes with same name: /imu_plugin`，gzserver 退出。
- **原因**：两辆车的插件 ROS 节点重名。
- **解决**：每个测试变体用全新 gzserver；不要在同一世界里 spawn 两份相同模型。

## 无头 gzserver "卡"在 Loading world file
- **现象**：日志停在 `Loading world file`，以为挂了。
- **原因**：软件渲染首次初始化着色器缓存很慢（WSL 上可达 30s+）。
- **解决**：等 `/clock` 话题出现再操作；日志不实时刷新不代表没在跑。

## `ros2 topic echo --once` 拿数据很慢
- **现象**：echo 一次要 1~2 秒。
- **原因**：CLI 启动开销。
- **影响**：测量运动距离时，echo 期间车可能仍在动，数值偏大。
- **解决**：测试脚本先显式发布零速 cmd_vel 停稳，再取数。
