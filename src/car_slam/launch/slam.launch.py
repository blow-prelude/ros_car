"""SLAM 建图: 复用 car_bringup/gazebo.launch.py (世界+机器人) + slam_toolbox 异步建图 + 建图 RViz

用法:
  export LIBGL_ALWAYS_SOFTWARE=1                   # WSL 软件渲染, 仓库约定手动 export
  ros2 launch car_slam slam.launch.py              # Gazebo + slam_toolbox + RViz
  ros2 launch car_slam slam.launch.py gui:=false   # 无头建图
  ros2 launch car_slam slam.launch.py slam_rviz:=false  # 不开建图 RViz
  # 注意: 本包的 rviz 参数叫 slam_rviz, 避免与被 include 的 gazebo.launch.py
  # 的同名参数 rviz 撞名被覆盖 (launch 同名参数穿透坑)

车不动图不长大, 另开终端遥控:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard

建完后存图:
  ros2 run nav2_map_server map_saver_cli -f ~/maps/home_world
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory('car_slam')
    bringup_share = get_package_share_directory('car_bringup')
    default_world = os.path.join(bringup_share, 'worlds', 'home_world.sdf')
    default_params = os.path.join(slam_share, 'config', 'slam_toolbox.yaml')
    default_rviz = os.path.join(slam_share, 'rviz', 'slam.rviz')

    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_gui = DeclareLaunchArgument('gui', default_value='true')
    declare_slam_rviz = DeclareLaunchArgument('slam_rviz', default_value='true')
    declare_with_camera = DeclareLaunchArgument('with_camera', default_value='false')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_spawn_x = DeclareLaunchArgument('spawn_x', default_value='-3.0')
    declare_spawn_y = DeclareLaunchArgument('spawn_y', default_value='-2.0')
    declare_spawn_yaw = DeclareLaunchArgument('spawn_yaw', default_value='0.0')

    # 世界 + 机器人; 建图 RViz 由本文件自己启动, 强制关掉 bringup 里那个
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',
            'with_camera': LaunchConfiguration('with_camera'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
    )

    # /scan + /odom -> /map 与 TF map->odom
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[default_params,
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        condition=IfCondition(LaunchConfiguration('slam_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        declare_world,
        declare_gui,
        declare_slam_rviz,
        declare_with_camera,
        declare_use_sim_time,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_yaw,

        bringup,
        slam_toolbox,
        rviz2,
    ])
