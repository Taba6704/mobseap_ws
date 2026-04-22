from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_share = FindPackageShare('usv_sim')

    xacro_file = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'usv.urdf.xacro'
    ])

    rviz_config_file = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'usv_sim.rviz'
    ])

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'robot_description': robot_description,
                }
            ]
        ),

        Node(
            package='usv_sim',
            executable='simple_usv_sim_node',
            name='simple_usv_sim_node',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'cmd_vel_topic': '/cmd_vel',
                    'odom_topic': '/odom',
                    'path_topic': '/sim_path',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'update_rate_hz': 20.0,
                    'cmd_timeout_sec': 0.5,
                    'publish_path': True,
                    'path_publish_every_n': 5,
                    'max_path_points': 1000,
                    'linear_scale': 1.0,
                    'angular_scale': 1.0,
                    'initial_x': 0.0,
                    'initial_y': 0.0,
                    'initial_yaw': 0.0,
                }
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])