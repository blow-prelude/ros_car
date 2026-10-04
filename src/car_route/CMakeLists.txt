cmake_minimum_required(VERSION 3.8)
project(car_route)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 自包含：信封编解码只用 vendored nlohmann 单头，不依赖系统 nlohmann-json-dev
set(NLOHMANN_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party")

add_executable(gateway src/gateway_node.cpp)
target_compile_features(gateway PUBLIC cxx_std_17)
target_include_directories(gateway PRIVATE ${NLOHMANN_INCLUDE_DIR})
ament_target_dependencies(gateway
  rclcpp
  std_msgs
)

add_executable(translator src/translator_node.cpp)
target_compile_features(translator PUBLIC cxx_std_17)
target_include_directories(translator PRIVATE ${NLOHMANN_INCLUDE_DIR})
ament_target_dependencies(translator
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
)

install(TARGETS gateway translator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
