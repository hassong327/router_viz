from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Launch file for PH-path VFG + PID control.

    Nodes:
    - dashboard_logger_node: Headless C++ dashboard logger (publishes DashboardSample + CSV)
    - ph_path_publisher: Publishes PH path from segment definitions
    - vfg_guidance_ph_node: PH path-based VFG guidance controller
    - pid_lateral_controller_ph_node: PID lateral controller
    """
    return LaunchDescription(
        [
            Node(
                package="qcar2_control_2",
                executable="dashboard_logger_node",
                name="dashboard_logger",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"update_rate_hz": 30.0},
                    {"save_directory": "./acc_plots"},
                    {"csv_log_enabled": True},
                ],
            ),
            Node(
                package="qcar2_control_2",
                executable="ph_path_publisher",
                name="ph_path_publisher",
                output="screen",
                emulate_tty=True,
                parameters=[
                    # For current SEGMENTS:
                    # - phase breaks [7, 12] split path into 3 parts (### boundaries)
                    # - stop breaks [6, 10] hold at waypoint "18" and "19"
                    {"phase_break_indices": [7, 12]},
                    {"phase_reach_hold_sec": 0.0},
                    {"stop_topic": "stop"},
                    {"stop_break_indices": [6, 10]},
                    {"stop_messages": ["stop1", "stop2"]},
                    {"stop_end_message": "stop3"},
                    {"stop_hold_sec": 3.0},
                ],
            ),
            Node(
                package="qcar2_control_2",
                executable="vfg_guidance_ph_node",
                name="vfg_guidance_ph_node",
                output="screen",
                emulate_tty=True,
            ),
            Node(
                package="qcar2_control_2",
                executable="longitudinal_controller_node",
                name="longitudinal_controller_node",
                output="screen",
                emulate_tty=True,
                parameters=[{
                    "speed_max": 0.6,
                    "speed_min": 0.3,
                    "curvature_low": 0.3,
                    "curvature_high": 0.7,
                    "max_acceleration": 0.2,
                    "max_deceleration": 0.4,
                    "speed_smooth_alpha": 0.5,
                }],
            ),
            Node(
                package="qcar2_control_2",
                executable="pid_lateral_controller_ph_node",
                name="pid_lateral_controller_ph_node",
                output="screen",
                emulate_tty=True,
            ),
        ]
    )
