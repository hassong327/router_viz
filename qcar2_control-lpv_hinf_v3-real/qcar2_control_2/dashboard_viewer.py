#!/usr/bin/env python3

import math
import os
import signal
import sys
from datetime import datetime
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path

from qcar2_msgs_2.msg import DashboardSample

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg
import pyqtgraph.exporters


# Tokyo Night theme
BG_COLOR = "#1a1b26"           # bg
TEXT_COLOR = "#a9b1d6"         # fg
GRID_COLOR = "#414868"         # bg_highlight
DIVIDER_COLOR = "#565f89"      # comment
PATH_COLOR = "#9ece6a"         # green
TRAJ_COLOR = "#f7768e"         # red
TARGET_SPEED_COLOR = "#7aa2f7"  # blue
ACTUAL_SPEED_COLOR = "#ff9e64"  # orange
STEER_COLOR = "#bb9af7"        # purple
CTE_COLOR = "#7dcfff"          # cyan
WAITING_COLOR = "#e0af68"      # yellow


pg.setConfigOption("background", BG_COLOR)
pg.setConfigOption("foreground", TEXT_COLOR)
pg.setConfigOptions(antialias=True, useOpenGL=False)


def _style_plot(p: pg.PlotItem, title: str, ylabel: str, xlabel: Optional[str] = None) -> None:
    p.setTitle(title, color=TEXT_COLOR, size="10pt")
    p.showGrid(x=True, y=True, alpha=0.3)
    p.getAxis("left").setLabel(ylabel, color=TEXT_COLOR)
    p.getAxis("left").setPen(pg.mkPen(GRID_COLOR))
    p.getAxis("bottom").setPen(pg.mkPen(GRID_COLOR))
    p.getAxis("left").setTextPen(pg.mkPen(TEXT_COLOR))
    p.getAxis("bottom").setTextPen(pg.mkPen(TEXT_COLOR))
    if xlabel is not None:
        p.getAxis("bottom").setLabel(xlabel, color=TEXT_COLOR)


class DashboardViewer(Node):
    def __init__(self, app: QtWidgets.QApplication) -> None:
        super().__init__("dashboard_viewer")

        self.declare_parameter("sample_topic", "/dashboard/sample")
        self.declare_parameter("path_topic", "/planning/local_path_path")
        self.declare_parameter("plot_xaxis", "time")  # time | distance
        self.declare_parameter("display_rotation_deg", 0.0)
        self.declare_parameter("save_directory", "./acc_plots")
        self.declare_parameter("auto_save_on_complete", True)
        self.declare_parameter("max_points", 200000)

        self._sample_topic = str(self.get_parameter("sample_topic").value)
        self._path_topic = str(self.get_parameter("path_topic").value)
        self._plot_xaxis = str(self.get_parameter("plot_xaxis").value)
        self._display_yaw = math.radians(float(self.get_parameter("display_rotation_deg").value))
        self._cos_yaw = math.cos(self._display_yaw)
        self._sin_yaw = math.sin(self._display_yaw)
        self._save_directory = str(self.get_parameter("save_directory").value)
        self._auto_save_on_complete = bool(self.get_parameter("auto_save_on_complete").value)
        self._max_points = int(self.get_parameter("max_points").value)

        self._app = app

        self._sample_received = False
        self._path_received = False
        self._completed = False
        self._png_saved = False
        self._map_autorange_enabled = False

        # Topic readiness checkboxes
        self._topic_ready = {
            "TF": False,
            "Map": False,
            "Path": False,
            "CMD": False,
        }

        # Time-series buffers
        self._t_x: List[float] = []
        self._speed_target: List[float] = []
        self._speed_actual: List[float] = []
        self._steer_deg: List[float] = []
        self._cte_cm: List[float] = []

        self._traj_x: List[float] = []
        self._traj_y: List[float] = []

        self._path_x: List[float] = []
        self._path_y: List[float] = []

        self._cte_running_sum: float = 0.0
        self._cte_count: int = 0

        # Subscriptions
        self._sub_sample = self.create_subscription(
            DashboardSample, self._sample_topic, self._sample_cb, 10
        )
        self._sub_path = self.create_subscription(
            Path, self._path_topic, self._path_cb, 10
        )

        # GUI
        self._build_ui()

        # Render timer (~30 Hz refresh, decoupled from data rate)
        self._render_timer = QtCore.QTimer()
        self._render_timer.timeout.connect(self._render)
        self._render_timer.start(33)

        # ROS spin via Qt timer
        self._spin_timer = QtCore.QTimer()
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(5)

        self.get_logger().info("dashboard_viewer started - waiting for topics...")

    def _build_ui(self) -> None:
        self._win = pg.GraphicsLayoutWidget(show=True, title="ACC Control Dashboard")
        self._win.resize(1300, 720)
        self._win.setBackground(BG_COLOR)

        x_label = "distance [m]" if self._plot_xaxis == "distance" else "time [s]"

        # Col 0: Speed (row 0), Steering (row 1), CTE (row 2)
        self._p_speed = self._win.addPlot(row=0, col=0)
        _style_plot(self._p_speed, "Speed", "speed [m/s]")
        self._curve_speed_target = self._p_speed.plot(
            pen=pg.mkPen(TARGET_SPEED_COLOR, width=1.8), name="target"
        )
        self._curve_speed_actual = self._p_speed.plot(
            pen=pg.mkPen(ACTUAL_SPEED_COLOR, width=1.8), name="actual"
        )
        self._p_speed.addLegend(offset=(-10, 10))
        self._p_speed.legend.setLabelTextColor(TEXT_COLOR)
        self._p_speed.legend.addItem(self._curve_speed_target, "target")
        self._p_speed.legend.addItem(self._curve_speed_actual, "actual")

        self._p_steer = self._win.addPlot(row=1, col=0)
        _style_plot(self._p_steer, "Steering Angle", "angle [deg]")
        self._curve_steer = self._p_steer.plot(pen=pg.mkPen(STEER_COLOR, width=1.8))
        self._p_steer.addLine(y=0, pen=pg.mkPen(GRID_COLOR, width=0.8))

        self._p_cte = self._win.addPlot(row=2, col=0)
        _style_plot(self._p_cte, "Cross-Track Error", "error [cm]", xlabel=x_label)
        self._curve_cte = self._p_cte.plot(pen=pg.mkPen(CTE_COLOR, width=1.8))
        self._p_cte.addLine(y=0, pen=pg.mkPen(GRID_COLOR, width=0.8))

        # 2cm-step thin white horizontal guides, redrawn on y-range change.
        self._cte_step_cm: float = 2.0
        self._cte_grid_lines: List[pg.InfiniteLine] = []
        self._cte_grid_pen = pg.mkPen(QtGui.QColor(255, 255, 255, 90), width=0.5)
        cte_vb = self._p_cte.getViewBox()
        if cte_vb is not None:
            cte_vb.sigYRangeChanged.connect(self._refresh_cte_grid)

        self._cte_stats = pg.TextItem("", color=TEXT_COLOR, anchor=(0, 0))
        self._cte_stats.setFont(QtGui.QFont("monospace", 8))
        self._p_cte.addItem(self._cte_stats, ignoreBounds=True)

        # Col 1: Map spanning rows 0..2
        self._p_map = self._win.addPlot(row=0, col=1, rowspan=3)
        _style_plot(self._p_map, "Path vs Trajectory", "y [m]", xlabel="x [m]")
        self._p_map.setAspectLocked(True)
        self._curve_path = self._p_map.plot(
            pen=pg.mkPen(PATH_COLOR, width=2.5), name="path"
        )
        self._curve_traj = self._p_map.plot(
            pen=pg.mkPen(TRAJ_COLOR, width=1.5), name="vehicle"
        )
        self._p_map.addLegend(offset=(-10, 10))
        self._p_map.legend.setLabelTextColor(TEXT_COLOR)
        self._p_map.legend.addItem(self._curve_path, "path")
        self._p_map.legend.addItem(self._curve_traj, "vehicle")

        self._waiting_text = pg.TextItem(
            "Waiting for topics...", color=WAITING_COLOR, anchor=(0.5, 0.5)
        )
        f = QtGui.QFont()
        f.setPointSize(16)
        f.setBold(True)
        self._waiting_text.setFont(f)
        self._p_map.addItem(self._waiting_text, ignoreBounds=True)

        self._waiting_status = pg.TextItem("", color=TEXT_COLOR, anchor=(0.5, 0.5))
        sf = QtGui.QFont("monospace")
        sf.setPointSize(10)
        self._waiting_status.setFont(sf)
        self._p_map.addItem(self._waiting_status, ignoreBounds=True)

        # Seed an initial range so the waiting text has a meaningful center
        # before any path/trajectory data arrives.
        self._p_map.setRange(xRange=(-1.0, 1.0), yRange=(-1.0, 1.0), padding=0)

        # Layout proportions
        self._win.ci.layout.setColumnStretchFactor(0, 1)
        self._win.ci.layout.setColumnStretchFactor(1, 1)

    def _format_topic_status(self) -> str:
        lines = []
        for name in ("TF", "Map", "Path", "CMD"):
            mark = "[OK]" if self._topic_ready[name] else "[  ]"
            lines.append(f"{mark} {name}")
        return "\n".join(lines)

    def _position_waiting_text(self) -> None:
        """Center the waiting overlay text in the map plot's current viewbox."""
        vb = self._p_map.getViewBox()
        if vb is None:
            return
        (xmin, xmax), (ymin, ymax) = vb.viewRange()
        xc = 0.5 * (xmin + xmax)
        yc = 0.5 * (ymin + ymax)
        dy = ymax - ymin
        self._waiting_text.setPos(xc, yc + 0.06 * dy)
        self._waiting_status.setPos(xc, yc - 0.10 * dy)

    def _refresh_cte_grid(self, _vb=None, y_range=None) -> None:
        if y_range is None:
            vb = self._p_cte.getViewBox()
            if vb is None:
                return
            (_, _), (ymin, ymax) = vb.viewRange()
        else:
            ymin, ymax = y_range
        step = self._cte_step_cm
        # Safety cap on the number of lines to draw (prevents runaway when the
        # view is auto-ranged to a very wide span before data settles).
        max_lines = 200
        span = ymax - ymin
        if step <= 0.0 or span <= 0.0 or span / step > max_lines:
            for line in self._cte_grid_lines:
                line.setVisible(False)
            return
        k_lo = int(math.ceil(ymin / step))
        k_hi = int(math.floor(ymax / step))
        targets = [k * step for k in range(k_lo, k_hi + 1) if k != 0]
        # Grow the pool if needed.
        while len(self._cte_grid_lines) < len(targets):
            line = pg.InfiniteLine(angle=0, pen=self._cte_grid_pen, movable=False)
            line.setZValue(-10)
            self._p_cte.addItem(line, ignoreBounds=True)
            self._cte_grid_lines.append(line)
        for line, y in zip(self._cte_grid_lines, targets):
            line.setPos(y)
            line.setVisible(True)
        for line in self._cte_grid_lines[len(targets):]:
            line.setVisible(False)

    def _rotate(self, x: float, y: float) -> Tuple[float, float]:
        return (
            self._cos_yaw * x - self._sin_yaw * y,
            self._sin_yaw * x + self._cos_yaw * y,
        )

    def _path_cb(self, msg: Path) -> None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        rotated = [self._rotate(x, y) for x, y in pts]
        self._path_x = [p[0] for p in rotated]
        self._path_y = [p[1] for p in rotated]
        self._topic_ready["Map"] = True
        if not self._path_received:
            self._path_received = True
            self.get_logger().info("Path topic received")

    def _sample_cb(self, msg: DashboardSample) -> None:
        if msg.tf_ok:
            self._topic_ready["TF"] = True
        if msg.motor_cmd_ok:
            self._topic_ready["CMD"] = True
        if msg.guidance_ok:
            self._topic_ready["Path"] = True

        if not self._sample_received:
            self._sample_received = True
            self.get_logger().info("DashboardSample topic received")

        # Don't start filling time-series until TF (= valid vehicle pose) is up.
        if not msg.tf_ok:
            return

        x_axis = msg.distance_travelled_m if self._plot_xaxis == "distance" else msg.elapsed_sec
        self._t_x.append(x_axis)
        self._speed_target.append(msg.target_speed_mps)
        self._speed_actual.append(msg.actual_speed_mps)
        self._steer_deg.append(math.degrees(msg.steering_rad))

        cte_cm = msg.cross_track_error_m * 100.0
        self._cte_cm.append(cte_cm)
        self._cte_running_sum += abs(cte_cm)
        self._cte_count += 1

        xd, yd = self._rotate(msg.vehicle_x_map, msg.vehicle_y_map)
        self._traj_x.append(xd)
        self._traj_y.append(yd)

        # Cap time-series memory (trajectory is left unbounded by request)
        if len(self._t_x) > self._max_points:
            trim = len(self._t_x) - self._max_points
            del self._t_x[:trim]
            del self._speed_target[:trim]
            del self._speed_actual[:trim]
            del self._steer_deg[:trim]
            del self._cte_cm[:trim]

        if msg.completed and not self._completed:
            self._completed = True
            self.get_logger().info("Run completed signal received")

    def _render(self) -> None:
        all_ready = all(self._topic_ready.values())

        if not all_ready:
            self._waiting_text.setText("Waiting for topics...")
            self._waiting_status.setText(self._format_topic_status())
            self._position_waiting_text()
            return

        # Hide waiting overlay once every topic is up
        if self._waiting_text.toPlainText():
            self._waiting_text.setText("")
            self._waiting_status.setText("")

        # The seed setRange() in _build_ui disables auto-range; re-enable it
        # the first time real data is available so the map fits path+trajectory.
        if not self._map_autorange_enabled:
            self._p_map.enableAutoRange(axis=pg.ViewBox.XYAxes, enable=True)
            self._p_map.getViewBox().setAutoVisible(x=True, y=True)
            self._map_autorange_enabled = True

        if not self._sample_received:
            return

        self._curve_speed_target.setData(self._t_x, self._speed_target)
        self._curve_speed_actual.setData(self._t_x, self._speed_actual)
        self._curve_steer.setData(self._t_x, self._steer_deg)
        self._curve_cte.setData(self._t_x, self._cte_cm)

        if self._path_x:
            self._curve_path.setData(self._path_x, self._path_y)
        self._curve_traj.setData(self._traj_x, self._traj_y)

        if self._cte_count > 0:
            avg_err = self._cte_running_sum / self._cte_count
            status = "completed" if self._completed else "running"
            self._cte_stats.setText(f"avg: {avg_err:.2f} cm  ({status})")
            if self._t_x:
                view_box = self._p_cte.getViewBox()
                if view_box is not None:
                    (xmin, _), (_, ymax) = view_box.viewRange()
                    self._cte_stats.setPos(xmin, ymax)

        if self._completed and self._auto_save_on_complete and not self._png_saved:
            self._save_png()

    def _save_png(self) -> None:
        if self._png_saved:
            return
        try:
            os.makedirs(self._save_directory, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filepath = os.path.join(self._save_directory, f"acc_plot_{timestamp}.png")
            exporter = pg.exporters.ImageExporter(self._win.scene())
            exporter.parameters()["width"] = 1600
            exporter.export(filepath)
            self._png_saved = True
            self.get_logger().info(f"Plot saved: {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to save plot: {e}")

    def _spin_once(self) -> None:
        rclpy.spin_once(self, timeout_sec=0.0)


def main() -> None:
    rclpy.init()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv)
    node = DashboardViewer(app)

    # Qt event loop blocks Python's SIGINT delivery. Install handlers that
    # call app.quit() so Ctrl+C terminates immediately (the _spin_timer at
    # 5 ms keeps the interpreter responsive enough to deliver the signal).
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())

    try:
        app.exec_()
    finally:
        node._save_png()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
