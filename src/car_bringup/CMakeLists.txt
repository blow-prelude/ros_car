cmake_minimum_required(VERSION 3.8)
project(car_bringup)

find_package(ament_cmake REQUIRED)

# models/: 世界里 <include> 的 ground_plane 与 sun(Gazebo 自带模型的本仓副本)。
# 随包装载是为了既不依赖 ~/.gazebo/models 的手工拷贝、也不依赖在线模型库,
# 原因与踩坑经过见 package.xml 里 gazebo_ros 导出的注释。
install(
  DIRECTORY launch worlds config models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
