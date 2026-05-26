import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Launch file for PH-path VFG + LPV H-infinity control.

    Nodes:
    - dashboard_logger_node: Headless C++ dashboard logger (publishes DashboardSample + CSV)
    - ph_path_publisher: Publishes PH path from segment definitions
    - vfg_guidance_ph_node: PH path-based VFG guidance controller
    - longitudinal_controller_node: Curvature-based longitudinal speed planner
    - h_inf_lateral_controller_node: LPV H-infinity lateral controller

    Shutdown behavior:
    - Ctrl+C in this terminal cascades shutdown to all nodes.
    - If ANY node exits (Ctrl+C, crash, or natural termination), the whole
      launch tears down — no zombies left behind.
    """
    controller_json_path = os.path.join(
        get_package_share_directory("qcar2_control_2"),
        "data",
        "lpv_hinf_qcar2_v7.json",
    )

    # Per-node SIGTERM/SIGKILL escalation. Defaults are 5s each (10s total
    # worst case). Shorten so unresponsive nodes are reaped quickly.
    sigterm_timeout = "3"
    sigkill_timeout = "2"

    dashboard_logger = Node(
        package="qcar2_control_2",
        executable="dashboard_logger_node",
        name="dashboard_logger",
        output="screen",
        emulate_tty=True,
        sigterm_timeout=sigterm_timeout,
        sigkill_timeout=sigkill_timeout,
        parameters=[
            {"update_rate_hz": 30.0},
            {"save_directory": "./acc_plots"},
            {"csv_log_enabled": True},
        ],
    )

    ph_path_publisher = Node(
        package="qcar2_control_2",
        executable="ph_path_publisher",
        name="ph_path_publisher",
        output="screen",
        emulate_tty=True,
        sigterm_timeout=sigterm_timeout,
        sigkill_timeout=sigkill_timeout,
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
    )

    vfg_guidance = Node(
        package="qcar2_control_2",
        executable="vfg_guidance_ph_node",
        name="vfg_guidance_ph_node",
        output="screen",
        emulate_tty=True,
        sigterm_timeout=sigterm_timeout,
        sigkill_timeout=sigkill_timeout,
        parameters=[{
            "k_e": 2.0,
            # Longitudinal lookahead: linear endpoint mapping from
            # (lookahead_speed_min → lookahead_min_distance)
            # to (lookahead_speed_max → lookahead_max_distance).
            "lookahead_min_distance": 0.1,
            "lookahead_max_distance": 1.5,
            "lookahead_speed_min":    0.6,
            "lookahead_speed_max":    1.2,
            # FF lookahead (H-inf κ_ff): d_ff = base + tau * v_cmd.
            # Fixed 0.1 m: base=0.1, tau=0.0 → speed-independent.
            "ff_lookahead_base_m": 0.2,
            "ff_lookahead_tau_s":  0.0,
        }],
    )

    longitudinal_controller = Node(
        package="qcar2_control_2",
        executable="longitudinal_controller_node",
        name="longitudinal_controller_node",
        output="screen",
        emulate_tty=True,
        sigterm_timeout=sigterm_timeout,
        sigkill_timeout=sigkill_timeout,
        parameters=[{
            # Loop rate: 200Hz -> 100Hz (matches vfg/H-inf 100Hz pipeline).
            "control_rate_hz": 100.0,
            "speed_max": 1.2,  # 1.0
            "speed_min": 0.6,
            "curvature_low": 0.1,
            "curvature_high": 0.3,
            "max_acceleration": 0.4, # 0.2
            "max_deceleration": 1.2,
            "speed_smooth_alpha": 0.8,
        }],
    )

    h_inf_controller = Node(
        package="qcar2_control_2",
        executable="h_inf_lateral_controller_node",
        name="h_inf_lateral_controller_node",
        output="screen",
        emulate_tty=True,
        sigterm_timeout=sigterm_timeout,
        sigkill_timeout=sigkill_timeout,
        parameters=[{
            "controller_json_path": controller_json_path,
            # Loop rate: 200Hz -> 100Hz. Must keep controller_dt
            # in lock-step (= 1 / control_rate_hz) so the Tustin
            # discretisation matches the actual update period.
            # NOTE: v7 JSON's dt_nominal=0.005 (200Hz). Running at
            # 100Hz means Tustin discretises at 2x the design dt;
            # phase warping stays small (omega_B_max * dt ~= 0.059)
            # but does not exactly match the synthesis assumption.
            "control_rate_hz": 100.0,
            "controller_dt":   0.010,
            "K_ff": 0.8,           # 0.7
            "rho_scale": 1.0,
            "output_gain": 1.0,   # 0.33
            "delta_max": 0.52,
            "wheelbase": 0.256,
            "steer_rate_limit_radps": 10.0,
            "speed_output_limit": 1.5,
            "steer_trim_rad": 0.0,    # -0.13
            "raw_encoder_counts": True,  # True: velocity[0]=counts/sec, False: velocity[0]=rad/s
            # v7 cascade outer PI on lateral error e_y (sign-flipped
            # internally so positive = vehicle LEFT, matching matlab
            # qcar2_params.sim convention). Set both to 0.0 to fall
            # back to pure v4-like single-loop behaviour.
            "Kp_outer": 0.2,
            "Ki_outer": 0.2,
        }],
    )

    nodes_with_names = [
        ("dashboard_logger",            dashboard_logger),
        ("ph_path_publisher",           ph_path_publisher),
        ("vfg_guidance_ph_node",        vfg_guidance),
        ("longitudinal_controller_node", longitudinal_controller),
        ("h_inf_lateral_controller_node", h_inf_controller),
    ]
    all_nodes = [node for _, node in nodes_with_names]

    # Cascade shutdown: if ANY node exits (Ctrl+C, crash, voluntary exit),
    # emit a Shutdown event so launch tears down every remaining node.
    cascade_handlers = [
        RegisterEventHandler(
            OnProcessExit(
                target_action=node,
                on_exit=[EmitEvent(event=Shutdown(
                    reason=f"{name} exited"))],
            )
        )
        for name, node in nodes_with_names
    ]

    return LaunchDescription(all_nodes + cascade_handlers)
