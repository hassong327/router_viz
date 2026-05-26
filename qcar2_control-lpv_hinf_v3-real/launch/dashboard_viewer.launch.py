"""Standalone PyQtGraph viewer launch (run on the host laptop)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="qcar2_control_2",
                executable="dashboard_viewer",
                name="dashboard_viewer",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"sample_topic": "/dashboard/sample"},
                    {"path_topic": "/planning/local_path_path"},
                    {"plot_xaxis": "time"},
                    {"display_rotation_deg": 0.0},
                    {"save_directory": "./acc_plots"},
                    {"auto_save_on_complete": True},
                ],
            ),
        ]
    )
