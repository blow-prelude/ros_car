# FAQS 索引

本项目（ROS 2 Humble + Gazebo Classic 11）调试过程中实际踩过的坑，按类别分文件，均为实测复现并验证过解法。

| 文件 | 内容 |
|---|---|
| [gazebo-model-path.md](gazebo-model-path.md) | Gazebo 模型解析类问题：`model://` 本地找不到 → 连在线模型库卡死（表现为"Gazebo 无响应"） |
| [gazebo-wsl.md](gazebo-wsl.md) | Gazebo 在 WSL2 上的环境类问题：渲染崩溃、模型路径卡死、端口占用、重复插件节点、孤儿节点 TF_OLD_DATA、加载慢 |
| [rviz-wsl.md](rviz-wsl.md) | RViz 在 WSL2 上的渲染问题：视口黄/青条纹垃圾（llvmpipe 多线程 tile bug） |
| [ros2-launch.md](ros2-launch.md) | launch/构建类问题：robot_description 传参、include 参数撞名、install 旧拷贝、命令行自杀 |
| [urdf-physics.md](urdf-physics.md) | URDF/物理仿真类问题：多余接触点、重心配平、锁轮极限环、插件标签名、IMU 伪影 |

背景与完整参数记录见 [../sim_car_spec.md](../sim_car_spec.md)。
