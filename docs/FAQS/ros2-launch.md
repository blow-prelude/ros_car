# ROS 2 launch / 构建类

## robot_state_publisher 启动即崩（Error document empty）
- **现象**：launch 里 RSP 报 `Error document empty ... Unable to initialize urdf::model`，退出码 -6。
- **原因**：`parameters=[{'robot_description': [替换列表]}]` 会把**命令文本原样**传给 RSP，xacro 根本没执行。
- **解决**：必须用 `ParameterValue(Command(['xacro ', ...]), value_type=str)` 包裹（官方 URDF 教程写法）。



## `pkill -f` 把自己的终端杀了
- **现象**：执行 `pkill -f gzserver` / `pkill -f "ros2 launch"` 后整个命令无输出退出。
- **原因**：`-f` 匹配完整命令行，而当前 shell 的命令行里就含这些关键字。
- **解决**：用 `pkill -x gzserver`（精确进程名）或按 pid kill。

## plugin 的 XML 标签名写错不报错
- **现象**：`<max_wheel_accel>` 写了没效果，也不报错。
- **原因**：gazebo 插件对未知标签静默忽略，用默认值。
- **解决**：以插件源码为准核对标签名（本例正确名是 `max_wheel_acceleration`），不确定就 grep 源码或 `strings libxxx.so`。

## include 别的 launch 后，自己的同名参数被覆盖（节点悄悄不启动）
- **现象**：`slam.launch.py` 自己声明了 `rviz` 参数（默认 true）并 include `gazebo.launch.py`（它也声明 `rviz`、自己传了 `rviz:=false`），结果 `ros2 launch ... rviz:=true` 时 rviz2 节点**不启动且日志无任何输出**。
- **原因**：include 执行时同名 launch configuration 被写入 `false`，等访问到自己的节点时 `IfCondition(LaunchConfiguration('rviz'))` 已为假。launch 对此完全静默。
- **解决**：顶层参数起不撞名的名字（本包用 `slam_rviz`）；给 include 传参时避免与顶层用户参数重名。判断依据：节点未启动且日志零输出 → 优先怀疑参数撞名，`ros2 node list` 确认后改名即好。



