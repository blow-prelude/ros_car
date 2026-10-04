"""加载家庭场景与小车模型: gzserver/gzclient + robot_state_publisher + spawn + 可选 RViz

用法:
  ros2 launch car_bringup gazebo.launch.py                 # 带界面
  ros2 launch car_bringup gazebo.launch.py gui:=false      # 无头模式
  ros2 launch car_bringup gazebo.launch.py rviz:=true      # 同时开 RViz
  ros2 launch car_bringup gazebo.launch.py with_camera:=true
  ros2 launch car_bringup gazebo.launch.py localization:=false  # 不启动 IMU 滤波/EKF 融合
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = get_package_share_directory('car_bringup')
    default_world = os.path.join(bringup_share, 'worlds', 'home_world.sdf')
    default_rviz = os.path.join(bringup_share, 'config', 'car.rviz')

    # xacro 生成机器人描述; 摄像头默认关闭(WSL GPU 离屏渲染不稳)
    robot_description = ParameterValue(
        Command(['xacro ', FindPackageShare('car_description'),
                 '/urdf/car.urdf.xacro', ' with_camera:=',
                 LaunchConfiguration('with_camera')]),
        value_type=str)

   
    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_rviz = DeclareLaunchArgument('rviz', default_value='false')
    declare_with_camera = DeclareLaunchArgument('with_camera', default_value='false')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_localization = DeclareLaunchArgument('localization', default_value='true')
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='-3.0')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='-2.0')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='0.0')

    # Gazebo (gzserver + gzclient)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'),
            'launch', 'gazebo.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gui')}.items(),
    )

    # 机器人 TF (base_link -> 各传感器/轮子)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # 在客厅 (-3, -2) 出生, 抬高 1cm 自然落地
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', 'xuegecar',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', '0.01',
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
    )

    # IMU 互补滤波 + EKF 融合轮速里程计: /odometry/filtered 与 odom->base_footprint TF
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('car_localization'),
            'launch', 'localization.launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        condition=IfCondition(LaunchConfiguration('localization')),
    )

    # 可选 RViz
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        declare_world,
        declare_gui,
        declare_rviz,
        declare_with_camera,
        declare_use_sim_time,
        declare_localization,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_yaw,

        gazebo,
        robot_state_publisher,
        spawn_entity,
        localization,
        # rviz2,
    ])
