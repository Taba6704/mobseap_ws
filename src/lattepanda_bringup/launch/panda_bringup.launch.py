from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    vision_node = Node(
        package='vision',
        executable='yolo_object_detection',
        name='vision_node',
        output='screen'
    )

    range_node = Node(
        package='vision',
        executable='target_range_node',
        name='range_node',
        output='screen'
    )

    path_planning_node = Node(
        package='simple_path_planner',
        executable='simple_path_planner_node',
        name='path_planning_node',
        output='screen'
    )

    return LaunchDescription([
        vision_node,
        range_node,
        path_planning_node,
    ])
