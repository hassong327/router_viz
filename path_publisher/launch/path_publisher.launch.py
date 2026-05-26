from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="path_publisher",
            executable="path_publisher_node",
            name="path_publisher_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"publish_rate_hz": 30.0},
                {"frame_id": "map"},
                {"traj_id": 1},
                {"publish_markers": True},
                {"publish_path": True},
                {"marker_line_width": 0.05},
                {"arc_sample_count": 20},
            ],
        ),
    ])
