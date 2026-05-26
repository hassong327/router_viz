#!/usr/bin/env python3

import math
from typing import Dict, List, Optional, Sequence, Set, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Point, PoseStamped, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from qcar2_msgs_2.msg import PhQuintic, PhQuinticPath
from tf2_ros import Buffer, TransformException, TransformListener

from qcar2_control_2.bezier_path_publisher import WAYPOINTS, SEGMENTS


def _distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _normalize_2d(dx: float, dy: float) -> Tuple[float, float]:
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return (1.0, 0.0)
    return (dx / n, dy / n)


def _lerp(a: Tuple[float, float, float], b: Tuple[float, float, float], t: float) -> Tuple[float, float, float]:
    return (
        a[0] + t * (b[0] - a[0]),
        a[1] + t * (b[1] - a[1]),
        a[2] + t * (b[2] - a[2]),
    )


def _quadratic_bezier(
    p0: Tuple[float, float, float],
    p1: Tuple[float, float, float],
    p2: Tuple[float, float, float],
    t: float,
) -> Tuple[float, float, float]:
    mt = 1.0 - t
    mt2 = mt * mt
    t2 = t * t
    return (
        mt2 * p0[0] + 2.0 * mt * t * p1[0] + t2 * p2[0],
        mt2 * p0[1] + 2.0 * mt * t * p1[1] + t2 * p2[1],
        mt2 * p0[2] + 2.0 * mt * t * p1[2] + t2 * p2[2],
    )


def _approx_arc_length(
    start: Tuple[float, float, float],
    apex: Tuple[float, float, float],
    end: Tuple[float, float, float],
    samples: int = 40,
) -> float:
    samples = max(2, samples)
    total = 0.0
    prev = _quadratic_bezier(start, apex, end, 0.0)
    for i in range(1, samples):
        t = float(i) / float(samples - 1)
        cur = _quadratic_bezier(start, apex, end, t)
        total += _distance(prev, cur)
        prev = cur
    return total


class PhPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__("ph_path_publisher")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("traj_id_start", 1)

        self.declare_parameter("scale", 0.975)
        self.declare_parameter("qlabs_to_map_theta", 3.921592653589793)
        self.declare_parameter("qlabs_to_map_tx", -0.20687896)
        self.declare_parameter("qlabs_to_map_ty", -1.39737787)
        self.declare_parameter("translate_z", 0.0)

        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_line_width", 0.05)
        self.declare_parameter("publish_path", True)
        self.declare_parameter("line_sample_count", 10)
        self.declare_parameter("arc_sample_count", 20)

        # Sequential mode:
        # - path phases are split by phase_break_indices
        # - optional stop hold points are defined by stop_break_indices
        self.declare_parameter("sequential_mode", True)
        self.declare_parameter("phase_break_indices", [7, 12])
        self.declare_parameter("phase_reach_distance_m", 0.35)
        self.declare_parameter("phase_reach_hold_sec", 0.0)
        self.declare_parameter("loop_phases", False)
        self.declare_parameter("stop_topic", "stop")
        self.declare_parameter("stop_break_indices", [6, 10])
        self.declare_parameter("stop_messages", ["stop1", "stop2"])
        self.declare_parameter("stop_end_message", "stop3")
        self.declare_parameter("stop_hold_sec", 3.0)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._traj_id = int(self.get_parameter("traj_id_start").value)

        self._scale = float(self.get_parameter("scale").value)
        self._theta_rad = float(self.get_parameter("qlabs_to_map_theta").value)
        self._translate = (
            float(self.get_parameter("qlabs_to_map_tx").value),
            float(self.get_parameter("qlabs_to_map_ty").value),
            float(self.get_parameter("translate_z").value),
        )

        self._publish_markers = bool(self.get_parameter("publish_markers").value)
        self._marker_line_width = float(self.get_parameter("marker_line_width").value)
        self._publish_path = bool(self.get_parameter("publish_path").value)
        self._line_sample_count = int(self.get_parameter("line_sample_count").value)
        self._arc_sample_count = int(self.get_parameter("arc_sample_count").value)

        self._sequential_mode = bool(self.get_parameter("sequential_mode").value)
        self._phase_break_indices = self._sanitize_break_indices(
            self._parse_int_list(self.get_parameter("phase_break_indices").value), len(SEGMENTS)
        )
        self._phase_reach_distance_m = float(self.get_parameter("phase_reach_distance_m").value)
        self._phase_reach_hold_sec = float(self.get_parameter("phase_reach_hold_sec").value)
        self._loop_phases = bool(self.get_parameter("loop_phases").value)
        self._stop_topic = str(self.get_parameter("stop_topic").value)
        self._stop_break_indices = self._sanitize_stop_break_indices(
            self._parse_int_list(self.get_parameter("stop_break_indices").value), len(SEGMENTS)
        )
        self._stop_messages = self._parse_str_list(self.get_parameter("stop_messages").value)
        self._stop_end_message = str(self.get_parameter("stop_end_message").value)
        self._stop_hold_sec = float(self.get_parameter("stop_hold_sec").value)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._current_phase_idx = 0
        self._phase_reach_start_time: Optional[rclpy.time.Time] = None
        self._internal_stop_reach_start_time: Optional[rclpy.time.Time] = None
        self._final_stop_published = False
        self._completed_stop_break_indices: Set[int] = set()
        self._published_stop_break_indices: Set[int] = set()

        self._transformed_waypoints: Dict[str, Tuple[float, float, float]] = {}
        self._all_segments_msg: List[PhQuintic] = []
        self._all_preview_points: List[Tuple[float, float, float]] = []
        self._phase_segment_defs: List[List[dict]] = []
        self._phase_segments_msg: List[List[PhQuintic]] = []
        self._phase_preview_points: List[List[Tuple[float, float, float]]] = []
        self._phase_end_keys: List[str] = []
        self._phase_ranges: List[Tuple[int, int]] = []
        self._phase_internal_stop_prefix_cache: Dict[Tuple[int, int], Tuple[List[PhQuintic], List[Tuple[float, float, float]]]] = {}

        self._rebuild_cache(reset_phase=True)

        self.add_on_set_parameters_callback(self._on_parameters)

        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(1e-3, publish_rate)
        self._timer = self.create_timer(period, self._on_timer)

        self._pub_ph = self.create_publisher(PhQuinticPath, "/planning/local_path_ph", 10)
        self._pub_marker = self.create_publisher(Marker, "/planning/local_path_marker", 10)
        self._pub_path = self.create_publisher(Path, "/planning/local_path_path", 10)
        self._pub_stop = self.create_publisher(String, self._stop_topic, 10)

        if self._sequential_mode:
            self.get_logger().info(
                f"PH path publisher started: sequential mode ON, phases={len(self._phase_segment_defs)}, breaks={self._phase_break_indices}"
            )
        else:
            self.get_logger().info(f"PH path publisher started: full path ({len(self._all_segments_msg)} segments)")

    def _parse_int_list(self, raw_value) -> List[int]:
        if isinstance(raw_value, (list, tuple)):
            out: List[int] = []
            for v in raw_value:
                try:
                    out.append(int(v))
                except (TypeError, ValueError):
                    continue
            return out
        return []

    def _parse_str_list(self, raw_value) -> List[str]:
        if isinstance(raw_value, (list, tuple)):
            out: List[str] = []
            for v in raw_value:
                out.append(str(v))
            return out
        return []

    def _sanitize_break_indices(self, breaks: List[int], total_segments: int) -> List[int]:
        if total_segments <= 1:
            return []
        valid = sorted({b for b in breaks if 0 < b < total_segments})
        return valid

    def _sanitize_stop_break_indices(self, breaks: List[int], total_segments: int) -> List[int]:
        if total_segments <= 0:
            return []
        # Stop breaks are split indices in [1, total_segments].
        valid = sorted({b for b in breaks if 0 < b <= total_segments})
        return valid

    def _split_segments_by_breaks(self, segments: Sequence[dict], breaks: List[int]) -> List[List[dict]]:
        if not segments:
            return []
        if not breaks:
            return [list(segments)]

        chunks: List[List[dict]] = []
        start = 0
        for b in breaks:
            chunk = list(segments[start:b])
            if chunk:
                chunks.append(chunk)
            start = b
        tail = list(segments[start:])
        if tail:
            chunks.append(tail)
        return chunks

    def _apply_transform_to_waypoints(
        self, waypoints: Dict[str, Tuple[float, float, float]]
    ) -> Dict[str, Tuple[float, float, float]]:
        c = math.cos(self._theta_rad)
        s = math.sin(self._theta_rad)
        tx, ty, tz = self._translate

        out: Dict[str, Tuple[float, float, float]] = {}
        for key, (x, y, z) in waypoints.items():
            xs = x * self._scale
            ys = y * self._scale
            zs = z * self._scale
            xr = c * xs - s * ys
            yr = s * xs + c * ys
            out[key] = (xr + tx, yr + ty, zs + tz)
        return out

    def _build_ph_segments_for(
        self,
        seg_defs: Sequence[dict],
        waypoints: Dict[str, Tuple[float, float, float]],
    ) -> List[PhQuintic]:
        out: List[PhQuintic] = []

        for seg in seg_defs:
            seg_type = seg["type"]
            msg = PhQuintic()

            if seg_type == "line":
                start = waypoints[seg["start"]]
                end = waypoints[seg["end"]]
                tdx, tdy = _normalize_2d(end[0] - start[0], end[1] - start[1])
                arc_length = _distance(start, end)
                msg.start_tangent = Vector3(x=tdx, y=tdy, z=0.0)
                msg.end_tangent = Vector3(x=tdx, y=tdy, z=0.0)
            elif seg_type == "arc":
                start = waypoints[seg["start"]]
                apex = waypoints[seg["apex"]]
                end = waypoints[seg["end"]]
                sdx, sdy = _normalize_2d(apex[0] - start[0], apex[1] - start[1])
                edx, edy = _normalize_2d(end[0] - apex[0], end[1] - apex[1])
                arc_length = _approx_arc_length(start, apex, end, samples=40)
                msg.start_tangent = Vector3(x=sdx, y=sdy, z=0.0)
                msg.end_tangent = Vector3(x=edx, y=edy, z=0.0)
            else:
                self.get_logger().warn(f"Unknown segment type: {seg_type}, skipping")
                continue

            msg.start_point = Point(x=start[0], y=start[1], z=start[2])
            msg.end_point = Point(x=end[0], y=end[1], z=end[2])
            msg.branch = int(seg.get("branch", 0))
            msg.arc_length = float(max(1e-6, arc_length))
            out.append(msg)

        return out

    def _build_preview_points_for(self, seg_defs: Sequence[dict]) -> List[Tuple[float, float, float]]:
        pts: List[Tuple[float, float, float]] = []

        for idx, seg in enumerate(seg_defs):
            seg_type = seg["type"]
            if seg_type == "line":
                start = self._transformed_waypoints[seg["start"]]
                end = self._transformed_waypoints[seg["end"]]
                count = max(2, self._line_sample_count)
                local = [_lerp(start, end, float(i) / float(count - 1)) for i in range(count)]
            elif seg_type == "arc":
                start = self._transformed_waypoints[seg["start"]]
                apex = self._transformed_waypoints[seg["apex"]]
                end = self._transformed_waypoints[seg["end"]]
                count = max(3, self._arc_sample_count)
                local = [_quadratic_bezier(start, apex, end, float(i) / float(count - 1)) for i in range(count)]
            else:
                continue

            if idx > 0 and pts and local:
                local = local[1:]
            pts.extend(local)

        return pts

    def _rebuild_cache(self, reset_phase: bool) -> None:
        self._transformed_waypoints = self._apply_transform_to_waypoints(WAYPOINTS)

        self._all_segments_msg = self._build_ph_segments_for(SEGMENTS, self._transformed_waypoints)
        self._all_preview_points = self._build_preview_points_for(SEGMENTS)

        self._phase_ranges = []
        start_idx = 0
        for break_idx in self._phase_break_indices:
            self._phase_ranges.append((start_idx, break_idx))
            start_idx = break_idx
        self._phase_ranges.append((start_idx, len(SEGMENTS)))

        self._phase_segment_defs = self._split_segments_by_breaks(SEGMENTS, self._phase_break_indices)
        self._phase_segments_msg = []
        self._phase_preview_points = []
        self._phase_end_keys = []
        self._phase_internal_stop_prefix_cache = {}

        for phase_idx, phase in enumerate(self._phase_segment_defs):
            self._phase_segments_msg.append(self._build_ph_segments_for(phase, self._transformed_waypoints))
            self._phase_preview_points.append(self._build_preview_points_for(phase))
            self._phase_end_keys.append(str(phase[-1]["end"]))
            if phase_idx < len(self._phase_ranges):
                phase_start, phase_end = self._phase_ranges[phase_idx]
                for stop_break in self._stop_break_indices:
                    if phase_start < stop_break < phase_end:
                        local_end = stop_break - phase_start
                        prefix_defs = phase[:local_end]
                        prefix_msg = self._build_ph_segments_for(prefix_defs, self._transformed_waypoints)
                        prefix_pts = self._build_preview_points_for(prefix_defs)
                        self._phase_internal_stop_prefix_cache[(phase_idx, stop_break)] = (prefix_msg, prefix_pts)

        if reset_phase or self._current_phase_idx >= max(1, len(self._phase_segment_defs)):
            self._current_phase_idx = 0
        self._phase_reach_start_time = None
        self._internal_stop_reach_start_time = None
        self._final_stop_published = False
        self._completed_stop_break_indices.clear()
        self._published_stop_break_indices.clear()

    def _get_current_internal_stop_break(self) -> Optional[int]:
        if self._current_phase_idx < 0 or self._current_phase_idx >= len(self._phase_ranges):
            return None
        phase_start, phase_end = self._phase_ranges[self._current_phase_idx]
        for break_idx in self._stop_break_indices:
            if break_idx in self._completed_stop_break_indices:
                continue
            if phase_start < break_idx < phase_end:
                return break_idx
        return None

    def _get_stop_point_for_break(self, break_idx: int) -> Optional[Tuple[str, Tuple[float, float, float]]]:
        if break_idx <= 0 or break_idx > len(SEGMENTS):
            return None
        seg = SEGMENTS[break_idx - 1]
        end_key = str(seg["end"])
        if end_key not in self._transformed_waypoints:
            return None
        return (end_key, self._transformed_waypoints[end_key])

    def _publish_stop_event_for_break(self, break_idx: int) -> None:
        if break_idx in self._published_stop_break_indices:
            return
        if break_idx not in self._stop_break_indices:
            return
        msg_idx = self._stop_break_indices.index(break_idx)
        if msg_idx < 0 or msg_idx >= len(self._stop_messages):
            return
        stop_text = self._stop_messages[msg_idx]
        self._publish_stop_text(stop_text)
        self._published_stop_break_indices.add(break_idx)
        self.get_logger().info(f"Stop event published at break {break_idx}: {stop_text}")

    def _try_advance_phase(self) -> None:
        if not self._sequential_mode:
            return
        if not self._phase_segment_defs:
            return
        if self._current_phase_idx >= len(self._phase_end_keys):
            return

        try:
            tf = self._tf_buffer.lookup_transform(
                self._frame_id,
                self._base_frame,
                Time(),
            )
        except TransformException:
            self._phase_reach_start_time = None
            self._internal_stop_reach_start_time = None
            return

        vx = tf.transform.translation.x
        vy = tf.transform.translation.y
        now = self.get_clock().now()

        pending_stop_break = self._get_current_internal_stop_break()
        if pending_stop_break is not None:
            stop_target = self._get_stop_point_for_break(pending_stop_break)
            if stop_target is None:
                self._completed_stop_break_indices.add(pending_stop_break)
            else:
                stop_key, stop_pt = stop_target
                stop_dist = math.hypot(vx - stop_pt[0], vy - stop_pt[1])
                if stop_dist <= self._phase_reach_distance_m:
                    self._publish_stop_event_for_break(pending_stop_break)
                    if self._internal_stop_reach_start_time is None:
                        self._internal_stop_reach_start_time = now
                        return

                    held = (now - self._internal_stop_reach_start_time).nanoseconds * 1e-9
                    if held < self._stop_hold_sec:
                        return

                    self._completed_stop_break_indices.add(pending_stop_break)
                    self._internal_stop_reach_start_time = None
                    self._traj_id += 1
                    self.get_logger().info(
                        f"Internal stop hold complete at waypoint {stop_key} (break={pending_stop_break}, traj_id={self._traj_id})"
                    )
                    return
                else:
                    self._internal_stop_reach_start_time = None
                    return

        self._internal_stop_reach_start_time = None

        if len(self._phase_segment_defs) <= 1:
            return

        last_idx = len(self._phase_segment_defs) - 1
        end_key = self._phase_end_keys[self._current_phase_idx]
        if end_key not in self._transformed_waypoints:
            return

        end_pt = self._transformed_waypoints[end_key]
        dist = math.hypot(vx - end_pt[0], vy - end_pt[1])
        if dist <= self._phase_reach_distance_m:
            if self._phase_reach_start_time is None:
                self._phase_reach_start_time = now
                return

            held = (now - self._phase_reach_start_time).nanoseconds * 1e-9
            if held < self._phase_reach_hold_sec:
                return

            prev_phase = self._current_phase_idx
            if self._current_phase_idx < last_idx:
                self._current_phase_idx += 1
            elif self._loop_phases:
                self._current_phase_idx = 0
                self._final_stop_published = False
                self._completed_stop_break_indices.clear()
                self._published_stop_break_indices.clear()
            else:
                if not self._final_stop_published:
                    self._publish_stop_text(self._stop_end_message)
                    self._final_stop_published = True
                    self.get_logger().info(f"Final stop event published: {self._stop_end_message}")
                return

            self._traj_id += 1
            self._phase_reach_start_time = None
            if prev_phase < len(self._phase_break_indices):
                self._publish_stop_event_for_break(self._phase_break_indices[prev_phase])
            self.get_logger().info(
                f"PH phase switched: {prev_phase + 1} -> {self._current_phase_idx + 1} (traj_id={self._traj_id})"
            )
        else:
            self._phase_reach_start_time = None

    def _publish_stop_text(self, text: str) -> None:
        stop_msg = String()
        stop_msg.data = text
        self._pub_stop.publish(stop_msg)

    def _build_marker(self, points: Sequence[Tuple[float, float, float]]) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = "ph_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = max(0.001, self._marker_line_width)
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points]
        return marker

    def _build_path(self, points: Sequence[Tuple[float, float, float]]) -> Path:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._frame_id

        poses: List[PoseStamped] = []
        for p in points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position = Point(x=p[0], y=p[1], z=p[2])
            ps.pose.orientation.w = 1.0
            poses.append(ps)

        path.poses = poses
        return path

    def _on_parameters(self, params):
        transform_changed = False
        preview_changed = False
        split_changed = False
        stop_changed = False
        mode_changed = False

        new_frame_id = self._frame_id
        new_base_frame = self._base_frame
        new_scale = self._scale
        new_theta = self._theta_rad
        new_translate = list(self._translate)

        new_publish_markers = self._publish_markers
        new_marker_line_width = self._marker_line_width
        new_publish_path = self._publish_path
        new_line_sample_count = self._line_sample_count
        new_arc_sample_count = self._arc_sample_count

        new_sequential_mode = self._sequential_mode
        new_phase_break_indices = list(self._phase_break_indices)
        new_phase_reach_distance_m = self._phase_reach_distance_m
        new_phase_reach_hold_sec = self._phase_reach_hold_sec
        new_loop_phases = self._loop_phases
        new_stop_break_indices = list(self._stop_break_indices)
        new_stop_messages = list(self._stop_messages)
        new_stop_end_message = self._stop_end_message
        new_stop_hold_sec = self._stop_hold_sec

        for param in params:
            if param.name == "frame_id":
                new_frame_id = str(param.value)
            elif param.name == "base_frame":
                new_base_frame = str(param.value)
            elif param.name == "scale":
                new_scale = float(param.value)
                transform_changed = True
            elif param.name == "qlabs_to_map_theta":
                new_theta = float(param.value)
                transform_changed = True
            elif param.name == "qlabs_to_map_tx":
                new_translate[0] = float(param.value)
                transform_changed = True
            elif param.name == "qlabs_to_map_ty":
                new_translate[1] = float(param.value)
                transform_changed = True
            elif param.name == "translate_z":
                new_translate[2] = float(param.value)
                transform_changed = True
            elif param.name == "publish_markers":
                new_publish_markers = bool(param.value)
            elif param.name == "marker_line_width":
                new_marker_line_width = float(param.value)
            elif param.name == "publish_path":
                new_publish_path = bool(param.value)
            elif param.name == "line_sample_count":
                new_line_sample_count = int(param.value)
                preview_changed = True
            elif param.name == "arc_sample_count":
                new_arc_sample_count = int(param.value)
                preview_changed = True
            elif param.name == "sequential_mode":
                new_sequential_mode = bool(param.value)
                mode_changed = True
            elif param.name == "phase_break_indices":
                new_phase_break_indices = self._sanitize_break_indices(self._parse_int_list(param.value), len(SEGMENTS))
                split_changed = True
            elif param.name == "phase_reach_distance_m":
                new_phase_reach_distance_m = float(param.value)
            elif param.name == "phase_reach_hold_sec":
                new_phase_reach_hold_sec = float(param.value)
            elif param.name == "loop_phases":
                new_loop_phases = bool(param.value)
            elif param.name == "stop_break_indices":
                new_stop_break_indices = self._sanitize_stop_break_indices(
                    self._parse_int_list(param.value), len(SEGMENTS)
                )
                stop_changed = True
            elif param.name == "stop_messages":
                new_stop_messages = self._parse_str_list(param.value)
                stop_changed = True
            elif param.name == "stop_end_message":
                new_stop_end_message = str(param.value)
            elif param.name == "stop_hold_sec":
                new_stop_hold_sec = float(param.value)

        self._frame_id = new_frame_id
        self._base_frame = new_base_frame
        self._scale = new_scale
        self._theta_rad = new_theta
        self._translate = tuple(new_translate)

        self._publish_markers = new_publish_markers
        self._marker_line_width = new_marker_line_width
        self._publish_path = new_publish_path
        self._line_sample_count = max(2, new_line_sample_count)
        self._arc_sample_count = max(3, new_arc_sample_count)

        self._sequential_mode = new_sequential_mode
        self._phase_break_indices = new_phase_break_indices
        self._phase_reach_distance_m = max(0.01, new_phase_reach_distance_m)
        self._phase_reach_hold_sec = max(0.0, new_phase_reach_hold_sec)
        self._loop_phases = new_loop_phases
        self._stop_break_indices = new_stop_break_indices
        self._stop_messages = new_stop_messages
        self._stop_end_message = new_stop_end_message
        self._stop_hold_sec = max(0.0, new_stop_hold_sec)

        if transform_changed or preview_changed or split_changed or stop_changed or mode_changed:
            self._rebuild_cache(reset_phase=(transform_changed or split_changed or stop_changed or mode_changed))
            self._traj_id += 1

        return SetParametersResult(successful=True)

    def _on_timer(self) -> None:
        self._try_advance_phase()

        if self._sequential_mode and self._phase_segments_msg:
            pending_stop_break = self._get_current_internal_stop_break()
            cache_key = (self._current_phase_idx, pending_stop_break) if pending_stop_break is not None else None
            if cache_key is not None and cache_key in self._phase_internal_stop_prefix_cache:
                segments_msg, preview_points = self._phase_internal_stop_prefix_cache[cache_key]
            else:
                segments_msg = self._phase_segments_msg[self._current_phase_idx]
                preview_points = self._phase_preview_points[self._current_phase_idx]
        else:
            segments_msg = self._all_segments_msg
            preview_points = self._all_preview_points

        msg = PhQuinticPath()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.traj_id = int(self._traj_id)
        msg.segments = segments_msg
        self._pub_ph.publish(msg)

        if self._publish_markers:
            self._pub_marker.publish(self._build_marker(preview_points))
        if self._publish_path:
            self._pub_path.publish(self._build_path(preview_points))


def main() -> None:
    rclpy.init()
    node = PhPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
