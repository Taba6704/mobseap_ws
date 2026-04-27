from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    vision_node = Node(
        package='vision',
        executable='yolo_object_detection',
        name='vision_node',
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        parameters=[{
            'model_path': '/home/mobseap/models/YOLOv6_Nano-R2_COCO_512x288.rvc4.tar.xz',
            'sports_ball_conf_min': 0.30,
            'person_conf_min': 0.35,
            'prefer_sports_ball': True,
            'show_only_selected_target': True,
        }]
    )

    range_node = Node(
        package='vision',
        executable='target_range_node',
        name='range_node',
        output='screen',
        respawn=True,
        respawn_delay=5.0
    )

    path_planning_node = Node(
        package='simple_path_planner',
        executable='simple_path_planner_node',
        name='path_planning_node',
        output='screen',
        respawn=True,
        respawn_delay=5.0
    )

    return LaunchDescription([
        TimerAction(period=2.0, actions=[vision_node]),
        TimerAction(period=5.0, actions=[range_node]),
        TimerAction(period=8.0, actions=[path_planning_node]),
    ])