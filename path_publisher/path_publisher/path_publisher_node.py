#!/usr/bin/env python3

import ast
import json
import math
import os
from typing import Dict, List, Sequence, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PoseStamped, Vector3
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from qcar2_msgs.msg import PhQuintic, PhQuinticPath


def _normalize_2d(dx: float, dy: float) -> Tuple[float, float]:
    n = math.hypot(dx, dy)
    if n < 1e-9:
        return (1.0, 0.0)
    return (dx / n, dy / n)


def _de_casteljau_2d(points: Sequence[Tuple[float, float]], u: float) -> Tuple[float, float]:
    """Evaluate an arbitrary-degree 2D Bezier curve at parameter u."""
    pts = [[float(p[0]), float(p[1])] for p in points]
    while len(pts) > 1:
        pts = [
            [
                (1.0 - u) * pts[i][0] + u * pts[i + 1][0],
                (1.0 - u) * pts[i][1] + u * pts[i + 1][1],
            ]
            for i in range(len(pts) - 1)
        ]
    return (pts[0][0], pts[0][1])


def _approx_bezier_arc_length(points: Sequence[Tuple[float, float]], samples: int = 40) -> float:
    """Numeric arc length of a 2D Bezier defined by its control polygon."""
    samples = max(2, samples)
    total = 0.0
    px, py = _de_casteljau_2d(points, 0.0)
    for i in range(1, samples):
        u = float(i) / float(samples - 1)
        cx, cy = _de_casteljau_2d(points, u)
        total += math.hypot(cx - px, cy - py)
        px, py = cx, cy
    return total


def _weld_segments(
    ctrl_segments: Sequence[Sequence[Tuple[float, float]]]
) -> List[List[Tuple[float, float]]]:
    """Force C0 continuity by snapping each segment start to the previous end."""
    welded = [[tuple(p) for p in seg] for seg in ctrl_segments]
    for i in range(1, len(welded)):
        welded[i][0] = welded[i - 1][-1]
    return welded


# Quintic Bezier control points in MAP frame (6 points per segment, measured).
# Segment boundaries are near-coincident; _weld_segments enforces exact equality
# (segment[n].end == segment[n+1].start, with segment[n].end as the master).
_CONTROL_POINTS_RAW: List[List[Tuple[float, float]]] = [
    [
        (-1.2764, -0.5935),
        (-0.8654, -1.0458),
        (-0.4016, -1.0635),
        (0.2149, -1.0752),
        (0.7551, -1.0693),
        (1.1309, -1.0693),
    ],
    [
        (1.125, -1.059),
        (1.6945, -1.1589),
        (1.9353, -0.9298),
        (2.1936, -0.6302),
        (2.2582, -0.4011),
        (2.2523, -0.0428),
    ],
    [
        (2.2465, -0.0502),
        (2.2523, 0.6489),
        (2.2523, 1.2069),
        (2.2641, 1.7709),
        (2.2582, 2.4053),
        (2.2582, 2.8517),
    ],
    [
        (2.2465, 2.8561),
        (2.223, 3.5786),
        (2.1936, 4.1249),
        (2.5518, 4.777),
        (1.1309, 4.3717),
        (0.027, 4.4539),
    ],
    [
        (-0.0023, 4.4231),
        (-0.6775, 4.4524),
        (-1.294, 4.6052),
        (-1.7109, 4.4466),
        (-2.0984, 4.0295),
        (-1.9575, 2.8664),
    ],
    [
        (-1.9634, 2.8532),
        (-1.9693, 1.6901),
        (-1.9634, 1.1027),
        (-2.1219, -0.0369),
        (-1.617, -0.3306),
        (-1.2823, -0.595),
    ],
]

# C0-welded version actually published (seg[i].start := seg[i-1].end).
CONTROL_POINTS: List[List[Tuple[float, float]]] = _weld_segments(_CONTROL_POINTS_RAW)


def _parse_courses_text(text: str):
    """Parse a courses file as JSON, falling back to a Python literal."""
    text = text.strip()
    if not text:
        raise ValueError("course file is empty")
    try:
        return json.loads(text)
    except (ValueError, json.JSONDecodeError):
        return ast.literal_eval(text)


def _select_course(data, course_id: int):
    """Pick one course out of the top-level container by its number.

    Top-level is a dict mapping number -> course, e.g. {"1": [...], "2": [...]}.
    A bare list is also accepted and treated either as a list of courses
    (1-based numbering) or, if it already looks like a single course, returned
    as-is when course_id == 1.
    """
    if isinstance(data, dict):
        for key in (str(course_id), course_id):
            if key in data:
                return data[key]
        raise KeyError(
            f"course {course_id} not found; available: {sorted(map(str, data.keys()))}"
        )
    if isinstance(data, list):
        idx = course_id - 1  # numbers are 1-based
        if 0 <= idx < len(data):
            return data[idx]
        raise IndexError(
            f"course {course_id} out of range; file has {len(data)} courses"
        )
    raise TypeError(f"unexpected top-level type in course file: {type(data).__name__}")


def _course_to_segments(
    course, points_per_curve: int
) -> List[List[Tuple[float, float]]]:
    """Convert a course (list of curves) into segments of (x, y) tuples.

    Each curve must carry exactly ``points_per_curve`` control points. Curves
    share endpoints (segment[i].start == segment[i-1].end); the actual welding
    happens later via _weld_segments.
    """
    if not isinstance(course, (list, tuple)):
        raise TypeError(f"course must be a list of curves, got {type(course).__name__}")
    segments: List[List[Tuple[float, float]]] = []
    for i, curve in enumerate(course):
        pts = [(float(p[0]), float(p[1])) for p in curve]
        if len(pts) != points_per_curve:
            raise ValueError(
                f"curve {i} has {len(pts)} control points, expected {points_per_curve}"
            )
        segments.append(pts)
    if not segments:
        raise ValueError("course contains no curves")
    return segments


def load_control_points(
    course_file: str, course_id: int, points_per_curve: int
) -> List[List[Tuple[float, float]]]:
    """Load, select, and C0-weld a course from a JSON/TXT file."""
    if not os.path.isfile(course_file):
        raise FileNotFoundError(f"course file not found: {course_file}")
    with open(course_file, "r", encoding="utf-8") as fh:
        data = _parse_courses_text(fh.read())
    course = _select_course(data, course_id)
    segments = _course_to_segments(course, points_per_curve)
    return _weld_segments(segments)


class PathPublisher(Node):
    def __init__(self) -> None:
        super().__init__("path_publisher_node")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("traj_id", 1)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("publish_path", True)
        self.declare_parameter("marker_line_width", 0.05)
        self.declare_parameter("arc_sample_count", 20)
        self.declare_parameter("course_file", "")
        self.declare_parameter("course_id", 1)
        self.declare_parameter("points_per_curve", 6)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._traj_id = int(self.get_parameter("traj_id").value)
        self._publish_markers = bool(self.get_parameter("publish_markers").value)
        self._publish_path = bool(self.get_parameter("publish_path").value)
        self._marker_line_width = float(self.get_parameter("marker_line_width").value)
        self._arc_sample_count = max(3, int(self.get_parameter("arc_sample_count").value))

        course_file = str(self.get_parameter("course_file").value)
        course_id = int(self.get_parameter("course_id").value)
        points_per_curve = max(2, int(self.get_parameter("points_per_curve").value))

        if course_file:
            control_points = load_control_points(course_file, course_id, points_per_curve)
            self.get_logger().info(
                f"loaded course {course_id} from {course_file}: "
                f"{len(control_points)} curves x {points_per_curve} control points"
            )
        else:
            control_points = CONTROL_POINTS
            self.get_logger().info("no course_file given; using built-in default course")

        self._segments_msg = self._build_segments(control_points)
        self._preview_points = self._build_preview(control_points)

        self._pub_ph = self.create_publisher(PhQuinticPath, "/planning/local_path_ph", 10)
        self._pub_marker = self.create_publisher(Marker, "/planning/local_path_marker", 10)
        self._pub_path = self.create_publisher(Path, "/planning/local_path_path", 10)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self._timer = self.create_timer(1.0 / max(1e-3, rate), self._on_timer)

        self.get_logger().info(
            f"path_publisher started: {len(self._segments_msg)} exact-Bezier segments, frame={self._frame_id}"
        )

    def _build_segments(
        self, ctrl_segments: Sequence[Sequence[Tuple[float, float]]]
    ) -> List[PhQuintic]:
        out: List[PhQuintic] = []
        for cps in ctrl_segments:
            if len(cps) < 2:
                continue
            p0 = cps[0]
            pn = cps[-1]
            sdx, sdy = _normalize_2d(cps[1][0] - cps[0][0], cps[1][1] - cps[0][1])
            edx, edy = _normalize_2d(cps[-1][0] - cps[-2][0], cps[-1][1] - cps[-2][1])

            msg = PhQuintic()
            msg.start_point = Point(x=float(p0[0]), y=float(p0[1]), z=0.0)
            msg.end_point = Point(x=float(pn[0]), y=float(pn[1]), z=0.0)
            msg.start_tangent = Vector3(x=sdx, y=sdy, z=0.0)
            msg.end_tangent = Vector3(x=edx, y=edy, z=0.0)
            msg.branch = 0
            msg.arc_length = float(max(1e-6, _approx_bezier_arc_length(cps)))
            msg.control_points = [
                Point(x=float(x), y=float(y), z=0.0) for (x, y) in cps
            ]
            out.append(msg)
        return out

    def _build_preview(
        self, ctrl_segments: Sequence[Sequence[Tuple[float, float]]]
    ) -> List[Tuple[float, float, float]]:
        pts: List[Tuple[float, float, float]] = []
        count = self._arc_sample_count
        for idx, cps in enumerate(ctrl_segments):
            if len(cps) < 2:
                continue
            local: List[Tuple[float, float, float]] = []
            for i in range(count):
                u = float(i) / float(count - 1)
                x, y = _de_casteljau_2d(cps, u)
                local.append((x, y, 0.0))
            if idx > 0 and pts and local:
                local = local[1:]
            pts.extend(local)
        return pts

    def _on_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()

        msg = PhQuinticPath()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.traj_id = int(self._traj_id)
        msg.segments = self._segments_msg
        self._pub_ph.publish(msg)

        if self._publish_markers:
            self._pub_marker.publish(self._build_marker(stamp))
        if self._publish_path:
            self._pub_path.publish(self._build_path(stamp))

    def _build_marker(self, stamp) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self._frame_id
        marker.ns = "path_publisher"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = max(0.001, self._marker_line_width)
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.6
        marker.color.b = 1.0
        marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in self._preview_points]
        return marker

    def _build_path(self, stamp) -> Path:
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = self._frame_id
        poses: List[PoseStamped] = []
        for p in self._preview_points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position = Point(x=p[0], y=p[1], z=p[2])
            ps.pose.orientation.w = 1.0
            poses.append(ps)
        path.poses = poses
        return path


def main() -> None:
    rclpy.init()
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
