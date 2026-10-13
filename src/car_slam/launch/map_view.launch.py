"""静态地图查看: nav2 map_server 加载已建好的 .yaml 地图 + RViz 显示

用法:
  export LIBGL_ALWAYS_SOFTWARE=1                   # WSL 软件渲染, 仓库约定手动 export
  ros2 launch car_slam map_view.launch.py          # 默认加载 maps/local_map.yaml
  ros2 launch car_slam map_view.launch.py map:=/path/to/other.yaml

说明:
  - 纯离线显示, 不起 Gazebo/定位/建图, use_sim_time 固定 false。
  - map_server 是生命周期节点, 由 nav2_lifecycle_manager(autostart) 自动 configure+activate。
  - /map 为 transient_local, RViz 端 Map 显示项必须配 Transient Local 才收得到。
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory('car_slam')
    default_map = os.path.join(slam_share, 'maps', 'local_map.yaml')
    default_rviz = os.path.join(slam_share, 'rviz', 'map_view.rviz')

    declare_map = DeclareLaunchArgument('map', default_value=default_map)

    # 加载 occupancy grid 发布到 /map (transient_local, 无需订阅者即可 latch)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map'),
                     'use_sim_time': False}],
    )

    # 生命周期管理: autostart 依次 configure/activate map_server
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'use_sim_time': False,
                     'autostart': True,
                     'node_names': ['map_server']}],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz],
        # Mesa 23.2.1 llvmpipe 多线程光栅化 tile bug 的保险, 详见 docs/FAQS/rviz-wsl.md
        additional_env={'LP_NUM_THREADS': '1'},
    )

    return LaunchDescription([
        declare_map,
        map_server,
        lifecycle_manager,
        rviz2,
    ])
