# ROS 2 launch / 构建类

## robot_state_publisher 启动即崩（Error document empty）
- **现象**：launch 里 RSP 报 `Error document empty ... Unable to initialize urdf::model`，退出码 -6。
- **原因**：`parameters=[{'robot_description': [替换列表]}]` 会把**命令文本原样**传给 RSP，xacro 根本没执行。
- **解决**：必须用 `ParameterValue(Command(['xacro ', ...]), value_type=str)` 包裹（官方 URDF 教程写法）。

## 改了 xacro/launch 但行为没变
- **现象**：编辑 src/ 下的文件后，仿真行为和之前完全一样。
- **原因**：`$(find pkg)`、`<xacro:include>`、launch 文件全部解析到 **install/** 里的拷贝；没重新 colcon build 等于没改。
- **解决**：改完必须 `colcon build` 并重新 source。偶尔普通 rebuild 也不重拷 launch 文件——删掉 `build/<pkg>` 和 `install/<pkg>` 强制重建。

## `pkill -f` 把自己的终端杀了
- **现象**：执行 `pkill -f gzserver` / `pkill -f "ros2 launch"` 后整个命令无输出退出。
- **原因**：`-f` 匹配完整命令行，而当前 shell 的命令行里就含这些关键字。
- **解决**：用 `pkill -x gzserver`（精确进程名）或按 pid kill。

## plugin 的 XML 标签名写错不报错
- **现象**：`<max_wheel_accel>` 写了没效果，也不报错。
- **原因**：gazebo 插件对未知标签静默忽略，用默认值。
- **解决**：以插件源码为准核对标签名（本例正确名是 `max_wheel_acceleration`），不确定就 grep 源码或 `strings libxxx.so`。

## Bash 里 `A && B &` 的陷阱
- **现象**：`source ... && gzserver ... &` 后续命令找不到环境。
- **原因**：`&` 优先级低，把**整条 && 链**（含 source）都放到了后台。
- **解决**：source 单独一行，gzserver 另起一行再加 `&`。
