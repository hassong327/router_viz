#!/usr/bin/env python3

import math
from typing import List, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from qcar2_msgs_2.msg import BezierCurve


def _distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _de_casteljau(points: Sequence[Tuple[float, float, float]], u: float) -> Tuple[float, float, float]:
    # Generic Bezier evaluation using De Casteljau's algorithm.
    pts = [list(p) for p in points]
    while len(pts) > 1:
        next_pts = []
        for i in range(len(pts) - 1):
            x = (1.0 - u) * pts[i][0] + u * pts[i + 1][0]
            y = (1.0 - u) * pts[i][1] + u * pts[i + 1][1]
            z = (1.0 - u) * pts[i][2] + u * pts[i + 1][2]
            next_pts.append([x, y, z])
        pts = next_pts
    return (pts[0][0], pts[0][1], pts[0][2])


def _lerp(a: Tuple[float, float, float], b: Tuple[float, float, float], t: float) -> Tuple[float, float, float]:
    """Linear interpolation between two points."""
    return (
        a[0] + t * (b[0] - a[0]),
        a[1] + t * (b[1] - a[1]),
        a[2] + t * (b[2] - a[2]),
    )


def _quadratic_bezier(p0: Tuple[float, float, float], p1: Tuple[float, float, float],
                       p2: Tuple[float, float, float], t: float) -> Tuple[float, float, float]:
    """Evaluate quadratic Bezier curve at parameter t."""
    # B(t) = (1-t)²P0 + 2(1-t)tP1 + t²P2
    mt = 1.0 - t
    mt2 = mt * mt
    t2 = t * t
    return (
        mt2 * p0[0] + 2 * mt * t * p1[0] + t2 * p2[0],
        mt2 * p0[1] + 2 * mt * t * p1[1] + t2 * p2[1],
        mt2 * p0[2] + 2 * mt * t * p1[2] + t2 * p2[2],
    )


def _generate_line_points(start: Tuple[float, float, float], end: Tuple[float, float, float],
                          num_points: int) -> List[Tuple[float, float, float]]:
    """Generate points along a straight line."""
    points = []
    for i in range(num_points):
        t = i / max(1, num_points - 1)
        points.append(_lerp(start, end, t))
    return points


def _generate_arc_points(start: Tuple[float, float, float], apex: Tuple[float, float, float],
                         end: Tuple[float, float, float], num_points: int) -> List[Tuple[float, float, float]]:
    """Generate points along a smooth arc using quadratic Bezier."""
    points = []
    for i in range(num_points):
        t = i / max(1, num_points - 1)
        points.append(_quadratic_bezier(start, apex, end, t))
    return points


def _generate_path_control_points(waypoints: dict, segments: List[dict],
                                   line_density: int = 3,
                                   arc_density: int = 5) -> List[Tuple[float, float, float]]:
    """
    Generate control points for the entire path.

    Args:
        waypoints: dict mapping names to (x, y, z) coordinates
        segments: list of segment definitions
        line_density: number of points per straight segment
        arc_density: number of points per arc segment

    Returns:
        List of control points for a high-degree Bezier curve
    """
    all_points: List[Tuple[float, float, float]] = []

    for i, seg in enumerate(segments):
        seg_type = seg["type"]

        if seg_type == "line":
            start = waypoints[seg["start"]]
            end = waypoints[seg["end"]]
            points = _generate_line_points(start, end, line_density)

            # Skip first point if not first segment (to avoid duplicates)
            if i > 0 and len(all_points) > 0:
                points = points[1:]
            all_points.extend(points)

        elif seg_type == "arc":
            start = waypoints[seg["start"]]
            apex = waypoints[seg["apex"]]
            end = waypoints[seg["end"]]
            points = _generate_arc_points(start, apex, end, arc_density)

            # Skip first point if not first segment
            if i > 0 and len(all_points) > 0:
                points = points[1:]
            all_points.extend(points)

    return all_points


# Path definition in the QLabs coordinate frame.
WAYPOINTS = {
    "start": (-1.069, -1.0, 0.006),
    "1": (0.0, -1.100, 0.006),

    "2": (1.537, -1.100, 0.006),      # Corner 1 start
    "3": (2.221, -1.100, 0.006),      # Corner 1 apex
    "4": (2.225, -0.229, 0.006),      # Corner 1 end

    "5": (2.280, 3.761, 0.006),       # Corner 2 start
    "6": (2.280, 4.429, 0.006),       # Corner 2 apex
    "7": (1.537, 4.429, 0.006),       # Corner 2 end

    "8": (-1.109, 4.429, 0.006),      # Corner 3 start
    "9": (-1.958, 4.429, 0.006),      # Corner 3 apex
    "10": (-1.949, 3.513, 0.006),     # Corner 3 end

    "11": (-1.958, 1.673, 0.006),
    "12": (-1.958, 0.795, 0.006),
    "13": (-0.905, 0.795, 0.006),

    "14": (1.537, 0.795, 0.006),
    "15": (2.251, 0.795, 0.006),
    "16": (2.251, 1.673, 0.006),

    "17": (-1.957, -0.083, 0.006),
    "18": (0.900, 4.429, 0.006),
    "19": (-1.355, 0.795, 0.006),

    "end": (-1.693, -0.385, 0.006),
}

# Segment definition.
SEGMENTS = [
    {"type": "line", "start": "start", "end": "1"},
    {"type": "line", "start": "1", "end": "2"},         # Straight: start -> 1 -> 2

    {"type": "arc", "start": "2", "apex": "3", "end": "4"},  # Left turn, corner 1
    {"type": "line", "start": "4", "end": "5"},         # Straight: 4 -> 5

    {"type": "arc", "start": "5", "apex": "6", "end": "7"},  # Left turn, corner 2
    {"type": "line", "start": "7", "end": "18"},        # Straight: 7 -> 18
    {"type": "line", "start": "18", "end": "8"},

    {"type": "arc", "start": "8", "apex": "9", "end": "10"}, # Left turn, corner 3
    {"type": "line", "start": "10", "end": "11"},

    {"type": "arc", "start": "11", "apex": "12", "end": "19"},
    {"type": "line", "start": "19", "end": "13"},
    {"type": "line", "start": "13", "end": "14"},

    {"type": "arc", "start": "14", "apex": "15", "end": "16"},
    {"type": "line", "start": "16", "end": "5"},

    {"type": "arc", "start": "5", "apex": "6", "end": "7"},
    {"type": "line", "start": "7", "end": "8"},

    {"type": "arc", "start": "8", "apex": "9", "end": "10"},
    {"type": "line", "start": "10", "end": "12"},

    {"type": "arc", "start": "12", "apex": "17", "end": "end"},
]


class BezierPathPublisher(Node):
    def __init__(self) -> None:
        super().__init__("bezier_path_publisher")

        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("curve_id_start", 1)
        self.declare_parameter("scale", 0.975)
        self.declare_parameter("qlabs_to_map_theta", 0.60)
        self.declare_parameter("qlabs_to_map_tx", 0.40687896)
        self.declare_parameter("qlabs_to_map_ty", 1.39737787)
        self.declare_parameter("translate_z", 0.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("marker_sample_count", 160)
        self.declare_parameter("marker_line_width", 0.05)
        self.declare_parameter("publish_path", True)
        self.declare_parameter("path_sample_count", 160)

        # Path generation parameters.
        self.declare_parameter("line_density", 4)   # Points per straight segment.
        self.declare_parameter("arc_density", 6)    # Points per arc segment.

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._curve_id = int(self.get_parameter("curve_id_start").value)
        self._scale = float(self.get_parameter("scale").value)
        self._theta_rad = float(self.get_parameter("qlabs_to_map_theta").value)
        self._translate = (
            float(self.get_parameter("qlabs_to_map_tx").value),
            float(self.get_parameter("qlabs_to_map_ty").value),
            float(self.get_parameter("translate_z").value),
        )
        self._publish_markers = bool(self.get_parameter("publish_markers").value)
        self._marker_sample_count = int(self.get_parameter("marker_sample_count").value)
        self._marker_line_width = float(self.get_parameter("marker_line_width").value)
        self._publish_path = bool(self.get_parameter("publish_path").value)
        self._path_sample_count = int(self.get_parameter("path_sample_count").value)

        line_density = int(self.get_parameter("line_density").value)
        arc_density = int(self.get_parameter("arc_density").value)

        # Auto-generate control points from waypoints and segment definitions.
        self._raw_control_points = _generate_path_control_points(
            WAYPOINTS, SEGMENTS, line_density, arc_density
        )
        self._degree = len(self._raw_control_points) - 1
        self._control_points = self._apply_transform(self._raw_control_points)
        self._length_m = self._approx_length(self._control_points, samples=60)

        self.get_logger().info(f"Generated path with degree={self._degree} ({len(self._raw_control_points)} control points)")

        self.add_on_set_parameters_callback(self._on_parameters)

        publish_rate = float(self.get_parameter("publish_rate_hz").value)
        period = 1.0 / max(1e-3, publish_rate)
        self._timer = self.create_timer(period, self._on_timer)

        self._pub = self.create_publisher(BezierCurve, "/planning/local_path", 10)
        self._marker_pub = self.create_publisher(Marker, "/planning/local_path_marker", 10)
        self._path_pub = self.create_publisher(Path, "/planning/local_path_path", 10)
        self.get_logger().info("Bezier path publisher started")

    def _apply_transform(
        self, points: Sequence[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        # Apply scale -> yaw rotation -> translation in that order.
        c = math.cos(self._theta_rad)
        s = math.sin(self._theta_rad)
        tx, ty, tz = self._translate

        transformed: List[Tuple[float, float, float]] = []
        for x, y, z in points:
            xs = x * self._scale
            ys = y * self._scale
            zs = z * self._scale
            xr = c * xs - s * ys
            yr = s * xs + c * ys
            zr = zs
            transformed.append((xr + tx, yr + ty, zr + tz))
        return transformed

    def _approx_length(self, points: Sequence[Tuple[float, float, float]], samples: int = 50) -> float:
        if len(points) < 2:
            return 0.0
        if samples < 2:
            samples = 2
        length = 0.0
        prev = _de_casteljau(points, 0.0)
        for i in range(1, samples):
            u = float(i) / float(samples - 1)
            cur = _de_casteljau(points, u)
            length += _distance(prev, cur)
            prev = cur
        return length

    def _on_parameters(self, params):
        new_frame_id = self._frame_id
        transform_changed = False
        new_scale = self._scale
        new_theta_rad = self._theta_rad
        new_translate = list(self._translate)
        new_publish_markers = self._publish_markers
        new_marker_sample_count = self._marker_sample_count
        new_marker_line_width = self._marker_line_width
        new_publish_path = self._publish_path
        new_path_sample_count = self._path_sample_count
        new_line_density = int(self.get_parameter("line_density").value)
        new_arc_density = int(self.get_parameter("arc_density").value)
        density_changed = False

        for param in params:
            if param.name == "frame_id":
                new_frame_id = str(param.value)
            elif param.name == "scale":
                new_scale = float(param.value)
                transform_changed = True
            elif param.name == "qlabs_to_map_theta":
                new_theta_rad = float(param.value)
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
            elif param.name == "marker_sample_count":
                new_marker_sample_count = int(param.value)
            elif param.name == "marker_line_width":
                new_marker_line_width = float(param.value)
            elif param.name == "publish_path":
                new_publish_path = bool(param.value)
            elif param.name == "path_sample_count":
                new_path_sample_count = int(param.value)
            elif param.name == "line_density":
                new_line_density = int(param.value)
                density_changed = True
            elif param.name == "arc_density":
                new_arc_density = int(param.value)
                density_changed = True

        self._frame_id = new_frame_id
        self._scale = new_scale
        self._theta_rad = new_theta_rad
        self._translate = tuple(new_translate)
        self._publish_markers = new_publish_markers
        self._marker_sample_count = new_marker_sample_count
        self._marker_line_width = new_marker_line_width
        self._publish_path = new_publish_path
        self._path_sample_count = new_path_sample_count

        if density_changed:
            self._raw_control_points = _generate_path_control_points(
                WAYPOINTS, SEGMENTS, new_line_density, new_arc_density
            )
            self._degree = len(self._raw_control_points) - 1
            self._control_points = self._apply_transform(self._raw_control_points)
            self._curve_id += 1
            self._length_m = self._approx_length(self._control_points, samples=60)
        elif transform_changed:
            self._control_points = self._apply_transform(self._raw_control_points)
            self._curve_id += 1
            self._length_m = self._approx_length(self._control_points, samples=60)

        return SetParametersResult(successful=True)

    def _on_timer(self) -> None:
        msg = BezierCurve()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.degree = int(self._degree)
        msg.control_points = [Point(x=p[0], y=p[1], z=p[2]) for p in self._control_points]
        msg.length_m = float(self._length_m)
        msg.curve_id = int(self._curve_id)
        self._pub.publish(msg)

        if self._publish_markers:
            self._marker_pub.publish(self._build_marker())
        if self._publish_path:
            self._path_pub.publish(self._build_path())

    def _build_marker(self) -> Marker:
        count = max(2, self._marker_sample_count)
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self._frame_id
        marker.ns = "bezier_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = max(0.001, self._marker_line_width)
        marker.color.a = 1.0
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2

        points: List[Point] = []
        for i in range(count):
            u = float(i) / float(count - 1)
            x, y, z = _de_casteljau(self._control_points, u)
            points.append(Point(x=x, y=y, z=z))
        marker.points = points
        return marker

    def _build_path(self) -> Path:
        count = max(2, self._path_sample_count)
        samples: List[Tuple[float, float, float]] = []
        for i in range(count):
            u = float(i) / float(count - 1)
            samples.append(_de_casteljau(self._control_points, u))

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self._frame_id

        poses: List[PoseStamped] = []
        for i in range(count):
            x, y, z = samples[i]
            if i < count - 1:
                nx, ny, _ = samples[i + 1]
                dx = nx - x
                dy = ny - y
            else:
                px, py, _ = samples[i - 1]
                dx = x - px
                dy = y - py
            yaw = math.atan2(dy, dx) if (dx * dx + dy * dy) > 1e-12 else 0.0
            qz = math.sin(0.5 * yaw)
            qw = math.cos(0.5 * yaw)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            poses.append(pose)

        path.poses = poses
        return path


def main() -> None:
    rclpy.init()
    node = BezierPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
