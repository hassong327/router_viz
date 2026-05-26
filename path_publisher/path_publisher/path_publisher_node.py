#!/usr/bin/env python3

import math
from typing import List, Sequence, Tuple

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


# Quintic Bezier control points in MAP frame (6 points per segment, used as-is).
# The raw path is closed: last point of the last segment == first point of segment 0.
_CONTROL_POINTS_RAW: List[List[Tuple[float, float]]] = [
    [
        (-1.163, -0.8138),
        (-0.8635, -1.1545),
        (-0.1942, -1.1193),
        (0.3519, -1.0781),
        (1.0154, -1.0781),
        (1.6025, -1.0605),
    ],
    [
        (1.626, -1.0311),
        (2.0957, -0.8902),
        (2.3893, -0.526),
        (2.1955, -0.0443),
        (2.2308, 0.6371),
        (2.2601, 3.5566),
    ],
    [
        (2.2308, 3.163),
        (2.2425, 3.7681),
        (2.4304, 4.6551),
        (1.6143, 4.5023),
        (1.2561, 4.426),
        (0.7159, 4.4612),
    ],
    [
        (0.6866, 4.4847),
        (0.1934, 4.4671),
        (-0.476, 4.4377),
        (-1.3978, 4.6022),
        (-1.6855, 4.3966),
        (-1.9204, 3.8855),
    ],
    [
        (-1.9439, 3.8327),
        (-1.9497, 3.2864),
        (-1.9615, 2.4816),
        (-1.985, 0.3669),
        (-2.1259, -0.244),
        (-1.163, -0.8138),
    ],
]

# C0-welded version actually published (seg[i].start := seg[i-1].end).
CONTROL_POINTS: List[List[Tuple[float, float]]] = _weld_segments(_CONTROL_POINTS_RAW)


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

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._traj_id = int(self.get_parameter("traj_id").value)
        self._publish_markers = bool(self.get_parameter("publish_markers").value)
        self._publish_path = bool(self.get_parameter("publish_path").value)
        self._marker_line_width = float(self.get_parameter("marker_line_width").value)
        self._arc_sample_count = max(3, int(self.get_parameter("arc_sample_count").value))

        self._segments_msg = self._build_segments(CONTROL_POINTS)
        self._preview_points = self._build_preview(CONTROL_POINTS)

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
