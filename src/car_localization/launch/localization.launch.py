"""IMU 互补滤波 + EKF 融合轮速里程计。

数据流: /imu/data -> imu_filter -> /imu/data_filtered
        /odom + /imu/data_filtered -> ekf_filter_node -> /odometry/filtered
                                                      + TF odom->base_footprint

用法:
  ros2 launch car_localization localization.launch.py                    # 仿真时钟
  ros2 launch car_localization localization.launch.py use_sim_time:=false  # 实物
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_ekf = os.path.join(
        get_package_share_directory('car_localization'), 'config', 'ekf.yaml')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true')

    imu_filter = Node(
        package='car_localization',
        executable='imu_filter_node',
        name='imu_filter',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[default_ekf,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        declare_use_sim_time,
        imu_filter,
        ekf,
    ])
