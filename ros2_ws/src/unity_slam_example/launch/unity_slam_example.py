import math
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'unity_slam_example'
    package_dir = get_package_share_directory(package_name)
    perception_params = os.path.join(package_dir, 'config', 'perception.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_dir, 'config', 'nav2_obstacle_avoidance.yaml'),
            description='Nav2 parameters for the Unity robot'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'perception', default_value='true',
            description='Camera + lidar obstacle detection (boxes, labels, confidence)'),
        DeclareLaunchArgument(
            'publish_sim_odom', default_value='true',
            description='Derive simulated odometry from Unity TF; disable if another node publishes /odom'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_tcp_endpoint'), 'launch', 'endpoint.py')
            ),
        ),


        Node(
            package='rviz2',
            condition=IfCondition(LaunchConfiguration('rviz')),
            executable='rviz2',
            output='screen',
            arguments=['-d', os.path.join(package_dir, 'nav2_unity.rviz')],
            parameters=[{'use_sim_time':True}]
        ),

        Node(
            package=package_name,
            executable='tf_odometry',
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_sim_odom')),
            parameters=[{'use_sim_time': True}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': LaunchConfiguration('params_file'),
                # Its built-in manager can't skip nodes or set bond_timeout; ours below starts Nav2.
                'autostart': 'false',
            }.items()
        ),

        # bond_timeout 0 disables bond heartbeats. Activation waits for Unity's TF (until Play),
        # and the 4 s bond check then fails as /clock starts, aborting the whole bringup.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_unity',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'autostart': True,
                         'bond_timeout': 0.0,
                         # Jazzy's navigation_launch also starts route_server and
                         # docking_server; this robot uses neither, so they stay unconfigured.
                         # slam_toolbox first: the planner's global costmap needs its map -> odom.
                         'node_names': ['slam_toolbox',
                                        'controller_server', 'smoother_server',
                                        'planner_server', 'behavior_server',
                                        'velocity_smoother', 'collision_monitor',
                                        'bt_navigator', 'waypoint_follower']}]),

        # Robot camera (Unity RosCameraSensor): TurtleBot3 Waffle RealSense mount, then the
        # optical frame convention (z forward, x right, y down).
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='camera_link_tf',
            arguments=['--x', '0.064', '--y', '-0.065', '--z', '0.094',
                       '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
            parameters=[{'use_sim_time': True}]),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='camera_optical_tf',
            arguments=['--roll', str(-math.pi / 2), '--yaw', str(-math.pi / 2),
                       '--frame-id', 'camera_link', '--child-frame-id', 'camera_rgb_optical_frame'],
            parameters=[{'use_sim_time': True}]),

        *[Node(package=package_name, executable=executable, name=name, output='screen',
               condition=IfCondition(LaunchConfiguration('perception')),
               parameters=[perception_params, {'use_sim_time': True}])
          for executable, name in [('lidar_obstacles', 'lidar_obstacles'),
                                   ('camera_detector', 'camera_detector'),
                                   ('obstacle_fusion', 'obstacle_fusion')]],

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                # Its own configure/activate events never fired inside this combined launch.
                'use_lifecycle_manager': 'true',
            }.items()
        )
    ])
