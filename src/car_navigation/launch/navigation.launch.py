"""导航总入口: 世界+机器人+EKF(car_bringup) + AMCL/map_server + Nav2 规划控制 + 导航 RViz

用法:
  ros2 launch car_navigation navigation.launch.py                 # 完整导航栈(带 Gazebo 界面)
  ros2 launch car_navigation navigation.launch.py gui:=false nav_rviz:=false   # 无头
  ros2 launch car_navigation navigation.launch.py controller:=mppi             # 换局部控制器
  ros2 launch car_navigation navigation.launch.py sim:=false nav_start_delay:=0.0
      # 仿真栈已在别的终端跑着, 只起导航层(调参迭代时省一次 Gazebo 冷启动)

前置与约定(踩坑都在这里):
  * 必须从建图起点出生: spawn_x/y/yaw 默认 -3.0/-2.0/0.0, 该点即 map 帧原点(见
    docs/devel/2026-10-13-design-navigation-stack.md §3), 所以 AMCL 的 initial_pose 取 (0,0,0)
    就已经是真值附近。改了出生点就必须用 RViz 的 "2D Pose Estimate" 重新给定位姿。
  * 与通信层互斥: 本 launch 不含 car_route。导航时不要起 route.launch.py —— translator 的
    500ms watchdog 会每 100ms 往 /cmd_vel 灌全零, 与 nav2 的指令对打。
  * map->odom 由 AMCL 发布, 与 slam.launch.py 的 slam_toolbox 互斥, 两者不得同时启动。
  * 定位栈与导航栈**分两拍**启动(nav_start_delay -> nav_nodes_delay): 两套生命周期管理器
    在同一瞬间齐发 configure 时, 服务响应可能被丢(实测: 导航管理器配到 bt_navigator 时收到
    "failed to send response to /bt_navigator/change_state (timeout)", 随后永久阻塞且不再报错,
    表现是 controller_server 等 5 个节点停在 inactive、waypoint_follower/velocity_smoother 停在
    unconfigured)。加够延时分拍后不再复现。真卡住了就把导航层 Ctrl-C 重起一次(仿真栈不受影响)。

RViz 里点 "2D Goal Pose" 下发目标; 无头验收用:
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\
    "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 2.0}, orientation: {w: 1.0}}}}"
航点回归: ros2 run car_navigation nav_waypoints

参数:
  sim                true=一并启动 Gazebo 与机器人; false=接入已在跑的仿真栈
  world/gui/with_camera  透传给 car_bringup/gazebo.launch.py
  spawn_x/y/yaw      出生位姿(默认即建图起点, 勿随意改)
  map                栅格地图 yaml(默认 car_slam 的建图产物)
  controller         局部控制器档: dwb | mppi(对应 config/controller_<档>.yaml)
  nav_start_delay    定位栈(map_server+amcl)延后启动秒数(默认 20.0)
  nav_nodes_delay    导航节点 + 生命周期管理器延后启动秒数(默认 25.0, 必须 > nav_start_delay)
                     sim:=false(仿真栈已就绪)时两者都可给 3.0/6.0 之类的小值
  nav_rviz           导航 RViz 开关(本包参数名避开被 include 的 gazebo.launch.py 的 rviz,
                     同名会被它静默覆盖成 false, 见 docs/FAQS/ros2-launch.md)
  use_sim_time       仿真时钟(默认 true)
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, SetLaunchConfiguration, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory('car_navigation')
    bringup_share = get_package_share_directory('car_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    default_world = os.path.join(bringup_share, 'worlds', 'home_world.sdf')
    # 地图是建图层(car_slam)的产物, 这里只消费: 默认指向它, 需要时用 map:= 换
    default_map = os.path.join(get_package_share_directory('car_slam'), 'maps', 'local_map.yaml')
    default_localization_params = os.path.join(nav_share, 'config', 'nav2_localization.yaml')
    default_navigation_params = os.path.join(nav_share, 'config', 'nav2_navigation.yaml')
    default_rviz = os.path.join(nav_share, 'rviz', 'nav2.rviz')

    sim = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    localization_params = LaunchConfiguration('localization_params')
    navigation_params = LaunchConfiguration('navigation_params')
    nav_start_delay = LaunchConfiguration('nav_start_delay')
    nav_nodes_delay = LaunchConfiguration('nav_nodes_delay')

    def check_nav_delays(context):
        """启动时序防呆: 导航栈必须晚于定位栈, 否则两套生命周期管理器又回到齐发 configure
        (见文件头第三条), 直接拒绝启动比让人对着 inactive 状态猜要省事。"""
        loc_delay = float(LaunchConfiguration('nav_start_delay').perform(context))
        nav_delay = float(LaunchConfiguration('nav_nodes_delay').perform(context))
        if nav_delay <= loc_delay:
            raise RuntimeError(
                'nav_nodes_delay(%.1f) 必须大于 nav_start_delay(%.1f): 导航栈要等定位栈'
                '(map_server+amcl)起来之后再起' % (nav_delay, loc_delay))
        return []

    def select_controller(context):
        """控制器档位 -> 参数文件(边界校验 + 防呆)。

        launch 参数只有运行期才知道取值, 故用 OpaqueFunction 在 Python 侧查表:
        只接受表中列出的档位, 非法值直接拒绝启动 —— 否则拼出一个不存在的路径,
        controller_server 会因参数文件缺失而静默起不来, 现象隐蔽难查。
        新增一档(如 TEB)只需在这里加一行 + 放一个 config/controller_<档>.yaml。
        """
        choice = LaunchConfiguration('controller').perform(context).strip().lower()
        files = {'dwb': 'controller_dwb.yaml', 'mppi': 'controller_mppi.yaml'}
        if choice not in files:
            raise RuntimeError(
                "controller 只接受 %s, 收到 '%s'" % ('|'.join(sorted(files)), choice))
        return [SetLaunchConfiguration(
            'controller_params_file', os.path.join(nav_share, 'config', files[choice]))]

    # ---- 世界 + 机器人 + EKF 定位栈(odom->base_footprint) ----
    # 导航 RViz 由本文件自己起, 强制关掉 bringup 那个
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',
            'with_camera': LaunchConfiguration('with_camera'),
            'use_sim_time': use_sim_time,
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
        condition=IfCondition(sim),
    )

    # ---- 定位层: map_server(离线栅格图) + amcl(map->odom 的唯一发布者) ----
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_share, 'launch', 'localization_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': localization_params,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
    )

    # ---- 导航层: 节点列表与 remap 逐条对齐 nav2_bringup/navigation_launch.py ----
    # 为什么不直接 include 它: 它只吃单一 params_file, 换不了控制器插件(见设计文档 §4.2)。
    nav_nodes = [
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen',
            # 参数文件顺序即覆盖顺序: 先通用配置, 再按 controller:= 选中的控制器档
            parameters=[navigation_params,
                        LaunchConfiguration('controller_params_file'),
                        {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother', executable='smoother_server', name='smoother_server',
            output='screen',
            parameters=[navigation_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen',
            parameters=[navigation_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            output='screen',
            parameters=[navigation_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen',
            parameters=[navigation_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', output='screen',
            parameters=[navigation_params, {'use_sim_time': use_sim_time}],
        ),
        # 导航模式下 /cmd_vel 的发布者 + 停车兜底(velocity_timeout), 见设计文档 §7.2
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen',
            parameters=[navigation_params, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                         'node_names': ['controller_server', 'smoother_server',
                                        'planner_server', 'behavior_server', 'bt_navigator',
                                        'waypoint_follower', 'velocity_smoother']}],
        ),
    ]

    nav_rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('nav_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='true',
                              description='一并启动 Gazebo 与机器人(false=接入已在跑的仿真栈)'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('with_camera', default_value='false'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('controller', default_value='dwb',
                              description='局部控制器档: dwb | mppi'),
        DeclareLaunchArgument('nav_start_delay', default_value='3.0',
                              description='定位栈(map_server+amcl)延后启动秒数'),
        DeclareLaunchArgument('nav_nodes_delay', default_value='10.0',
                              description='导航节点与生命周期管理器延后启动秒数(须 > nav_start_delay)'),
        DeclareLaunchArgument('nav_rviz', default_value='true'),
        DeclareLaunchArgument('localization_params', default_value=default_localization_params),
        DeclareLaunchArgument('navigation_params', default_value=default_navigation_params),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='-3.0'),
        DeclareLaunchArgument('spawn_y', default_value='-2.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),

        # 控制器档位校验 + 参数文件选定, 必须在 controller_server 之前(它读这个配置)
        OpaqueFunction(function=select_controller),
        # 启动时序防呆(在参数声明之后、任何进程启动之前)
        OpaqueFunction(function=check_nav_delays),

        bringup,
        nav_rviz,
        # 分两拍: 先定位栈, 再导航栈(见文件头"前置与约定"第三条)
        TimerAction(period=nav_start_delay, actions=[localization]),
        TimerAction(period=nav_nodes_delay, actions=nav_nodes),
    ])
