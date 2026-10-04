cmake_minimum_required(VERSION 3.8)
project(car_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(imu_filter_node src/imu_filter_node.cpp)
target_compile_features(imu_filter_node PUBLIC cxx_std_17)
ament_target_dependencies(imu_filter_node
  rclcpp
  sensor_msgs
  tf2
)

install(TARGETS imu_filter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
