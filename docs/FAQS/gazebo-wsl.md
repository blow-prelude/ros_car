# Gazebo @ WSL2 环境类

## 两个 Gazebo 实例同时运行 → 加载页卡死/世界丢失/行为诡异
- **现象**：gzclient 卡在 "Preparing your world..."、场景缺模型、第二个实例的 gzserver 抢不到 11345 端口变成"无主"运行（物理和 ROS 插件正常、GUI 连不上）。
- **原因**：11345 主端口只能归一个实例；两个实例的 gztransport 互相干扰。
- **解决**：同一时间只跑一个 Gazebo 实例；确需并行时给每个实例设不同的 `GAZEBO_MASTER_URI`（如 http://localhost:11346）。GUI 现象诡异时先检查 `pgrep -x gzserver` 是不是不止一个。仅杀 gzserver 不够——残留的孤儿 ROS 节点同样致乱，见下条 TF_OLD_DATA。

## gzclient 打开后完全卡死（视角拖不动，无报错）
- **现象**：`ros2 launch car_bringup gazebo.launch.py` 一切正常加载完，但 Gazebo 窗口冻住、相机拖不动；无头模式（gui:=false）却一切正常。
- **原因**：软件渲染（LIBGL_ALWAYS_SOFTWARE=1）下 gzclient 每帧用 CPU 光栅化整车 STL 高模（controller 30 万面 + 电机 22 万面，整车 61 万三角），再加阴影多 pass 渲染，llvmpipe 直接被打满。gzserver 无头时不画网格所以没感觉。
- **解决**：①meshes/decimated/ 减面网格（整车 61 万 → 10 万面，URDF 已默认指向），原 CAD 高模保留在 meshes/ 顶层；②世界文件关阴影（`<scene><shadows>false</shadows></scene>`）。改完后 gzclient CPU 从满载降到 ~10%，RTF 0.93。
- **工具**：`pip3 install --user "trimesh==4.1.6" fast-simplification`（新版 trimesh 需 numpy≥2，Ubuntu 22.04 的 numpy 1.x 只能用旧版）；注意 `fast_simplification.simplify(v, f, r)` 的第三参是**要删除的面占比**（0.95 = 删 95%），不是保留占比。

## gzclient 画面定格 / 黑屏，FPS 和 RTF 恒为 0，但仿真实际在跑
- **现象**：世界"加载出来"后画面完全定格（首帧后不再刷新），FPS 0、Real Time Factor 0，机器人打了 cmd_vel 也"不动"（其实是画面没动）。此时无头检查一切正常：`ros2 topic hz /clock` ≈10Hz、odom 在走。左侧 World 树 Models 为空。**只在场景含小车（有 mesh visual）时出现**；空世界、home_world 不带小车都流畅。
- **原因**：URDF 视觉写的是 `package://car_description/meshes/...`，gzserver 做 URDF→SDF 转换时会自动改写成 `model://car_description/meshes/...`（Gazebo Classic 解析器行为，源码里看不到 `model://`）。`model://` 只按 `GAZEBO_MODEL_PATH` 解析，而 car_description 此前没有导出模型路径 → 网格 URI 全部解析失败（日志 `[Err] [Visual.cc] No mesh specified`、`[Wrn] [SystemPaths.cc] File or path does not exist [""]`）→ gzclient 场景加载卡死渲染管线：llvmpipe 工作线程 `/proc/<pid>/task/*/syscall` 持续 `running` 空转，帧永不上屏。同一份 mesh，RViz 走 ament 的 `package://` 一直正常——双 URI 体系正是这个坑的迷惑点。
- **鉴别**（与上一条"高模卡死"区分）：上一条 CPU 满载但画面仍在动（只是慢）；本条画面**零像素变化**、传输线程还在收位姿。实锤方法：`xwd -id <gzclient窗口id>` 隔 3 秒抓两帧做像素 diff，0 变化即定格。
- **解决**：`src/car_description/package.xml` 导出模型路径，gazebo_ros 的 GazeboRosPaths（gzserver/gzclient launch 都走它）自动追加：
  ```xml
  <export>
    <gazebo_ros gazebo_model_path="${prefix}/.."/>
  </export>
  ```
  必须写 `${prefix}`（会被替换成包 share 目录）；只写 `..` 会变成相对 CWD 的路径，无效。改完 `colcon build --packages-select car_description` 刷新 install。等价老办法是把 mesh 拷进 `~/.gazebo/models`（Gazebo 内置默认搜索路径，wpr_simulation2 就是这么活的），但 export 随包走、不依赖手工拷贝。
- **验证**：`tr '\0' '\n' < /proc/$(pgrep -x gzclient)/environ | grep GAZEBO_MODEL_PATH` 应含 `install/car_description/share`；修复后 FPS/RTF 恢复非 0，30s+ 不再定格。

## gzserver 启动即崩溃 / 空世界都加载不了
- **现象**：日志出现 `D3D12: Removing Device` 或直接 segfault，连 empty.world 都起不来。
- **原因**：WSL2 的 Intel D3D12 显卡驱动让 OGRE 离屏渲染崩溃。
- **解决**：`export LIBGL_ALWAYS_SOFTWARE=1` 强制软件渲染（ray/imu 传感器完全够用，摄像头也能工作只是慢）。
  注意这是**环境变量**、写用户 shell（`~/.bashrc`/`~/.zshrc`），`gazebo.launch.py` **并未代设**；2026-10-10 迁到原生 Ubuntu + NVIDIA 机器后该变量已移除且不再需要（硬件渲染正常），**若回到 WSL2 要自己加回来**。

## 摄像头传感器导致整个仿真崩溃
- **现象**：带 camera sensor 的模型一 spawn，gzserver 就死。
- **原因**：同上，GPU 离屏渲染；软件渲染下正常。
- **解决**：摄像头做成 xacro 开关（`with_camera:=true` 才启用），默认关闭。

## 新 gzserver 报 "Address already in use"
- **现象**：kill 旧 gzserver 后立刻重启，报端口 11345 被占。
- **原因**：端口释放有延迟。
- **解决**：kill 后等 3~5 秒，`ss -tln | grep 11345` 确认释放再启动。

## 无头 gzserver "卡"在 Loading world file
- **现象**：日志停在 `Loading world file`，以为挂了。
- **原因**：软件渲染首次初始化着色器缓存很慢（WSL 上可达 30s+）。
- **解决**：等 `/clock` 话题出现再操作；日志不实时刷新不代表没在跑。

## 残留孤儿节点导致 TF_OLD_DATA 恒定刷屏
- **现象**：启动仿真后终端持续刷 `TF_OLD_DATA ignoring data from the past for frame odom at time X.XX`（rviz2、slam_toolbox、ekf_node 三方同时打印），且时间戳恒定不变（如 27.23）。
- **原因**：ROS 域里存在两套栈。launch 进程被强杀/只清了部分进程时，slam_toolbox、ekf_node 等子进程残留为孤儿（`ros2 node list` 可见节点成对重复）。孤儿 slam_toolbox 的 `/clock` 冻结在 gzserver 死亡时刻，但 wall timer 仍以 50Hz 发布 map→odom（时间戳恒定），与现役 slam_toolbox 的 map→odom 交错到达，互相被判为"来自过去"。注意单栈下 slam_toolbox 车不动时 map→odom 时间戳恒定是**正常**行为（唯一发布者，恒定值被 TF 静默去重），不产生告警；出现刷屏必然是同边有第二个发布者。
- **解决**：全量清理后再启动。kill 名单必须包含**全部**节点进程名（`pkill -x` 要写全名，少打 `_node` 后缀会静默漏杀）：
  ```bash
  for n in gzserver gzclient rviz2 async_slam_toolbox_node ekf_node imu_filter_node robot_state_publisher; do pkill -x $n; done; sleep 5
  # 仍存活者补刀: pkill -9 -x <名>（孤儿常卡在不可中断态，SIGTERM 杀不动）
  ```
  启动前检查：`pgrep -x gzserver` 为空且 `ros2 node list` 无成对重复节点。
- **验证**：清理后单栈启动 50s，日志 TF_OLD_DATA 出现 0 次；/tf 监听确认每条边只有一条单调时间戳流。

## `ros2 topic echo --once` 拿数据很慢
- **现象**：echo 一次要 1~2 秒。
- **原因**：CLI 启动开销。
- **影响**：测量运动距离时，echo 期间车可能仍在动，数值偏大。
- **解决**：测试脚本先显式发布零速 cmd_vel 停稳，再取数。
