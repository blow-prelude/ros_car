"""通信层: gateway(TCP:10000 NDJSON 透传) + translator(信封 JSON <-> /cmd_vel、/odom)

用法:
  ros2 launch car_route route.launch.py               # 拉起 gateway + translator
  ros2 launch car_route route.launch.py port:=10001   # gateway 换 TCP 监听端口

前置: 仿真栈需另行启动 (ros2 launch car_bringup gazebo.launch.py),
本文件只负责通信层两个节点; Qt 面板连接 TCP:{port} 即可遥控遥测。
两节点均为真实时间语义 (TCP 心跳/cmd_vel 看门狗), 不设 use_sim_time。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_port = DeclareLaunchArgument('port', default_value='10000')

    # TCP <-> /wheels/uplink|downlink; port 是与 Qt 面板的耦合面, 暴露为 launch 参数
    gateway = Node(
        package='car_route',
        executable='gateway',
        name='gateway',
        output='screen',
        parameters=[{'port': LaunchConfiguration('port')}],
    )

    # /wheels/uplink|downlink <-> /cmd_vel|/odom; 各参数用节点内默认值, 不在此重复
    translator = Node(
        package='car_route',
        executable='translator',
        name='translator',
        output='screen',
    )

    return LaunchDescription([
        declare_port,

        gateway,
        translator,
    ])
