cmake_minimum_required(VERSION 3.8)
project(car_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

# 航点回归脚本: 文件名即可执行名(ROS 惯例, ros2 run 按可执行名精确匹配), 无 .py 后缀。
# 不写成 install(... RENAME ...) —— 那样装出来是拷贝而非 symlink, 改了脚本不重建会静默跑旧代码
# (同 docs/FAQS/ros2-launch.md 里 install 旧拷贝那条坑)。
install(PROGRAMS
  scripts/nav_waypoints
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
