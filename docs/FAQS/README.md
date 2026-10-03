# FAQS 索引

本项目（ROS 2 Humble + Gazebo Classic 11 @ WSL2）调试过程中实际踩过的坑，按类别分文件，均为实测复现并验证过解法。

| 文件 | 内容 |
|---|---|
| [gazebo-wsl.md](gazebo-wsl.md) | Gazebo 在 WSL2 上的环境类问题：渲染崩溃、端口占用、重复插件节点、加载慢 |
| [ros2-launch.md](ros2-launch.md) | launch/构建类问题：robot_description 传参、install 旧拷贝、命令行自杀 |
| [urdf-physics.md](urdf-physics.md) | URDF/物理仿真类问题：多余接触点、重心配平、锁轮极限环、插件标签名、IMU 伪影 |

背景与完整参数记录见 [../sim_car_spec.md](../sim_car_spec.md)。
