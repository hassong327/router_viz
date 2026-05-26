# qcar2_control_2

ROS 2 path-following control package for QCar2 in QLabs simulator.

Two independent control stacks are implemented:

| Stack | Path type | Lateral controller |
|---|---|---|
| **Bezier** | High-order Bezier curve | PID or LPV H-infinity |
| **PH** | Pythagorean Hodograph Quintic | PID or LPV H-infinity |

---

## 1. Package Structure

```text
qcar2_control_2/
├── include/qcar2_control_2/
│   ├── bezier.hpp              # Shared Bezier math (Vec2, evalBezier, …)
│   ├── ph_runtime.hpp          # PH quintic runtime (C++ library)
│   ├── lpv_hinf_runtime.hpp    # LPV H-infinity controller runtime
│   └── nlohmann/json.hpp       # JSON parser (header-only, vendored)
├── launch/
│   ├── bezier_vfg_control.launch.py                       # Bezier + PID
│   ├── ph_vfg_control.launch.py                           # PH + PID
│   ├── ph_vfg_h_infinity.launch.py                        # PH + H-infinity (primary)
│   ├── test_step_arc_bezier_control.launch.py             # Bezier step-arc test + PID
│   ├── test_step_arc_bezier_h_infinity_control.launch.py  # Bezier step-arc test + H-inf
│   ├── dashboard_viewer.launch.py                         # GUI viewer (separate process)
│   └── control_run_logger.py                              # Logger helper script
├── qcar2_control_2/
│   ├── bezier_path_publisher.py
│   ├── ph_path_publisher.py
│   ├── dashboard.py                # (legacy) integrated matplotlib dashboard
│   ├── dashboard_viewer.py         # PyQt5/pyqtgraph live viewer (subscribes DashboardSample)
│   ├── control_run_logger.py
│   └── test_step_arc_bezier_path_publisher.py
├── scripts/
│   ├── bezier_path_publisher
│   ├── ph_path_publisher
│   ├── dashboard
│   ├── dashboard_viewer
│   └── test_step_arc_bezier_path_publisher
├── src/
│   ├── vfg_guidance.cpp            # Bezier VFG guidance node
│   ├── vfg_guidance_ph.cpp         # PH VFG guidance node
│   ├── pid_controller.cpp          # PID lateral controller (Bezier)
│   ├── pid_controller_ph.cpp       # PID lateral controller (PH)
│   ├── h_inf_controller_bezier.cpp # LPV H-inf lateral controller (Bezier & PH)
│   ├── longitudinal_controller.cpp # Longitudinal (speed) controller
│   ├── dashboard_logger.cpp        # Headless C++ logger (DashboardSample + CSV)
│   ├── ph_runtime.cpp              # PH quintic runtime library
│   └── lpv_hinf_runtime.cpp        # LPV H-inf runtime library
├── data/
│   ├── lpv_hinf_qcar2_v1.json      # (legacy) SISO mixsyn synthesis
│   ├── lpv_hinf_qcar2_v2.json      # LPV H-inf, original W_e
│   ├── lpv_hinf_qcar2_v3.json      # LPV H-inf, W_e with LF-boost
│   └── lpv_hinf_qcar2_v3.1.json    # LPV H-inf, v3 refined (current default)
├── CMakeLists.txt
├── package.xml
└── setup.py
```

---

## 2. Overall Data Flow

### 2.1 Bezier + PID

```text
bezier_path_publisher ──► /planning/local_path (BezierCurve)
                                  │
                          vfg_guidance_node
                                  │
                          /vfg/lateral_guidance (LateralGuidance)
                                  │
                    ┌─────────────┴──────────────┐
          longitudinal_controller_node     pid_lateral_controller_node
                    │                              │
         /control/target_speed           /qcar2_motor_speed_cmd
```

### 2.2 PH + LPV H-infinity (primary stack)

```text
ph_path_publisher ──► /planning/local_path_ph (PhQuinticPath)
                                │
                       vfg_guidance_ph_node
                                │
                       /vfg/lateral_guidance (LateralGuidance)
                                │
               ┌────────────────┴────────────────┐
     longitudinal_controller_node     h_inf_lateral_controller_node
               │                              │   ▲
    /control/target_speed            /qcar2_joint (rho scheduling)
                                              │
                                  /qcar2_motor_speed_cmd (MotorCommands)
```

### 2.3 Dashboard (headless logger + optional GUI viewer)

```text
/planning/local_path_path ─┐
/vfg/lateral_guidance ─────┤
/qcar2_motor_speed_cmd ────┼──► dashboard_logger ──┬─► /dashboard/sample (DashboardSample)
/qcar2_joint ──────────────┤        (headless)     │
TF (map → base_link) ──────┘                       └─► CSV file (save_directory)

                          /dashboard/sample ──► dashboard_viewer (PyQt5 GUI, separate launch)
                                                       │
                                                       └─► PNG snapshot on exit
```

The logger runs inside every main launch file. The viewer is a **separate process** that you start in another terminal when you want live visualization — keeping the control stack headless-friendly.

### Custom messages (`qcar2_msgs_2`)

- `BezierCurve` — degree, control_points, length_m, curve_id
- `LateralGuidance` — heading_error_rad, cross_track_error_m, u_star, curvature_1pm, lookahead_*, ff_lookahead_curvature_1pm, w1/w2, status
- `PhQuintic` — start/end_point, start/end_tangent, arc_length, branch
- `PhQuinticPath` — header, traj_id, segments[]
- `DashboardSample` — aggregated per-tick snapshot used by dashboard logger/viewer
- (common interface) `qcar2_interfaces/MotorCommands` — motor_names[], values[]

---

## 3. Node Reference

### 3.1 `bezier_path_publisher` / `test_step_arc_bezier_path_publisher`

Generates high-order Bezier paths from waypoint/segment definitions and publishes them.

Publishes:
- `/planning/local_path` (`qcar2_msgs_2/BezierCurve`)
- `/planning/local_path_marker` (`visualization_msgs/Marker`)
- `/planning/local_path_path` (`nav_msgs/Path`)

Key parameters:
- `publish_rate_hz` (default `30.0`)
- `scale`, `qlabs_to_map_theta`, `qlabs_to_map_tx`, `qlabs_to_map_ty`

---

### 3.2 `ph_path_publisher`

Publishes multi-segment PH Quintic paths with phase/stop break support.

Publishes:
- `/planning/local_path_ph` (`qcar2_msgs_2/PhQuinticPath`)
- `/planning/local_path_ph_path` (`nav_msgs/Path`)

Key parameters:
- `phase_break_indices`, `phase_reach_hold_sec`
- `stop_topic`, `stop_break_indices`, `stop_messages`, `stop_end_message`, `stop_hold_sec`

---

### 3.3 `vfg_guidance_node` (Bezier)

Nearest-point search on a Bezier curve using De Casteljau subdivision. Computes VFG heading error, curvature, lookahead max curvature, and FF lookahead curvature.

Subscribes: `/planning/local_path`, TF
Publishes: `/vfg/lateral_guidance`, `/vfg/debug_markers` (optional)

Key parameters:
- `update_rate_hz` (default `100.0`)
- `a0` (default `0.2`) — VFG convergence weight
- `lookahead_preview_time` (default `1.3`), `lookahead_min_distance` (default `0.1`), `lookahead_max_distance` (default `1.2`)
- `ff_lookahead_distance` (default `0.13`) — fixed FF lookahead distance [m]

---

### 3.4 `vfg_guidance_ph_node` (PH)

Newton-based nearest-point search on PH quintic segments. FF lookahead distance is speed-adaptive.

Subscribes: `/planning/local_path_ph`, TF
Publishes: `/vfg/lateral_guidance`, `/vfg/debug_markers` (optional)

Key parameters:
- `update_rate_hz` (default `100.0`)
- `a0` (default `0.2`), `k_e` (default `1.0`)
- Lookahead curvature distance — linear endpoint mapping from speed to distance:
  - `lookahead_min_distance` (default `0.2`), `lookahead_max_distance` (default `1.5`)
  - `lookahead_speed_min` (default `0.6`), `lookahead_speed_max` (default `1.2`)
  - `lookahead_speed_min` → `lookahead_min_distance`, `lookahead_speed_max` → `lookahead_max_distance`; clamped at the endpoints.
- `ff_lookahead_base_m` (default `0.12`) — base FF lookahead distance [m]
- `ff_lookahead_tau_s` (default `0.16`) — speed-proportional FF lookahead time constant [s]
  - effective distance = `ff_lookahead_base_m + ff_lookahead_tau_s × speed`

---

### 3.5 `longitudinal_controller_node`

Computes target speed from lookahead curvature using a cubic smoothstep and publishes it. Also handles stop detection at the penultimate path point.

Subscribes: `/vfg/lateral_guidance`, `/planning/local_path`, `/planning/local_path_ph`
Publishes: `/control/target_speed` (`std_msgs/Float64`)

Key parameters:
- `speed_max` (default `1.0`), `speed_min` (default `0.3`)
- `curvature_low` (default `0.3`), `curvature_high` (default `0.7`)
- `max_acceleration` (default `0.2`), `max_deceleration` (default `0.4`)
- `speed_smooth_alpha` (default `0.8`)
- `stop_on_penultimate` (default `true`), `stop_distance_m` (default `0.1`)

---

### 3.6 `pid_lateral_controller_node` / `pid_lateral_controller_ph_node`

PID + feed-forward lateral controller.

Subscribes: `/vfg/lateral_guidance`, `/control/target_speed`
Publishes: `/qcar2_motor_speed_cmd` (`qcar2_interfaces/MotorCommands`)

Key parameters:
- `control_rate_hz` (default `200.0`)
- `kp`, `ki`, `kd`, `kff`, `wheelbase` (default `0.256`)
- `gain_schedule_enabled`, `gain_ref_speed`, `gain_min_speed`
- `steer_smooth_alpha`

---

### 3.7 `h_inf_lateral_controller_node` (LPV H-infinity)

LPV H-infinity lateral controller with polytopic interpolation. Scheduling parameter `ρ = |κ| × |v_measured|`, where `v_measured` comes from `/qcar2_joint` (falls back to `/control/target_speed` if not yet received).

Subscribes: `/vfg/lateral_guidance`, `/control/target_speed`, `/qcar2_joint`
Publishes: `/qcar2_motor_speed_cmd`

Key parameters:
- `control_rate_hz` (default `200.0`)
- `K_ff` (default `0.7`) — curvature feedforward gain
- `rho_scale` (default `1.0`), `output_gain` (default `0.35`)
- `delta_max` (default `0.52`), `wheelbase` (default `0.256`)
- `steer_rate_limit_radps` (default `10.0`)
- `speed_output_limit` (default `1.5`) — throttle command clamp [m/s]
- `steer_trim_rad` (default `0.0`) — steering bias offset [rad]
- `controller_json_path` — path to vertex matrices JSON (default: package share `data/lpv_hinf_qcar2_v3.1.json`)
- `gear_ratio` (default `0.0954`), `wheel_radius` (default `0.033`), `encoder_cpr` (default `2880.0`)
- `raw_encoder_counts` (default `true`) — `false`: `velocity[0]` is rad/s (ROS standard); `true`: counts/sec (QLabs raw)

---

### 3.8 `dashboard_logger_node` (C++, headless)

Headless logger included in every main launch file. Aggregates path / guidance / motor command / joint state / TF into a single `DashboardSample` message and writes a per-run CSV.

Subscribes: `/planning/local_path_path` (configurable), `/vfg/lateral_guidance`, `/qcar2_motor_speed_cmd`, `/qcar2_joint`, TF
Publishes: `/dashboard/sample` (`qcar2_msgs_2/DashboardSample`)

Key parameters:
- `update_rate_hz` (default `30.0`)
- `save_directory` (default `./acc_plots`), `csv_log_enabled` (default `true`)
- `path_topic`, `motor_cmd_topic`, `joint_topic`, `guidance_topic`, `sample_topic`
- `end_signal_topic` (default `stop`), `end_signal_message` (default `stop3`) — triggers CSV finalization
- `gear_ratio` (default `0.0954`), `wheel_radius` (default `0.033`), `encoder_cpr` (default `2880.0`)
- `frame_id` (default `map`), `base_frame` (default `base_link`), `tf_timeout_sec` (default `0.1`)

---

### 3.9 `dashboard_viewer` (Python, PyQt5 + pyqtgraph)

Live GUI viewer (Tokyo Night theme). Subscribes to `DashboardSample` from the logger — does not duplicate I/O. Designed to be launched on demand in a separate terminal.

Panels:
- **Map panel** — Path vs vehicle trajectory
- **Speed panel** — Target and actual speed
- **Steering panel** — Steering angle [deg]
- **CTE panel** — Cross-track error [cm] + running average

Saves PNG snapshot on process exit if `auto_save_on_complete=true`.

Subscribes: `/dashboard/sample`, `/planning/local_path_path` (configurable)

Key parameters:
- `sample_topic` (default `/dashboard/sample`), `path_topic` (default `/planning/local_path_path`)
- `plot_xaxis` (`time` | `distance`, default `time`)
- `display_rotation_deg` (default `0.0`)
- `auto_save_on_complete` (default `true`), `save_directory` (default `./acc_plots`)
- `max_points` (default `200000`) — sample buffer cap

### 3.10 `dashboard` (legacy)

The original integrated Python dashboard (matplotlib) remains available as `dashboard.py` for backward compatibility. New work should use the logger/viewer split above.

---

## 4. Launch Files

| Launch file | Path publisher | Guidance | Lateral controller |
|---|---|---|---|
| `bezier_vfg_control.launch.py` | `bezier_path_publisher` | `vfg_guidance_node` | `pid_lateral_controller_node` |
| `ph_vfg_control.launch.py` | `ph_path_publisher` | `vfg_guidance_ph_node` | `pid_lateral_controller_ph_node` |
| `ph_vfg_h_infinity.launch.py` | `ph_path_publisher` | `vfg_guidance_ph_node` | `h_inf_lateral_controller_node` |
| `test_step_arc_bezier_control.launch.py` | `test_step_arc_bezier_path_publisher` | `vfg_guidance_node` | `pid_lateral_controller_node` |
| `test_step_arc_bezier_h_infinity_control.launch.py` | `test_step_arc_bezier_path_publisher` | `vfg_guidance_node` | `h_inf_lateral_controller_node` |
| `dashboard_viewer.launch.py` | — | — | (GUI viewer only) |

All main launch files include `dashboard_logger_node` and `longitudinal_controller_node`.
`test_step_arc_*` launch files additionally run `control_run_logger`.
`dashboard_viewer.launch.py` is meant to be run separately when live GUI visualization is needed.

---

## 5. Build

```bash
colcon build --packages-select qcar2_control_2
source install/setup.bash
```

Dependencies (`package.xml`):
- `rclcpp`, `rclpy`, `tf2_ros`, `tf2_geometry_msgs`
- `geometry_msgs`, `nav_msgs`, `std_msgs`, `sensor_msgs`, `visualization_msgs`
- `qcar2_msgs_2`, `qcar2_interfaces`, `ament_index_cpp`
- `python3-numpy`, `python3-scipy`
- `python3-matplotlib` — for legacy `dashboard.py`
- `python3-pyqt5`, `python3-pyqtgraph` — for `dashboard_viewer`

---

## 6. Run

```bash
# PH path + LPV H-infinity (primary stack)
ros2 launch qcar2_control_2 ph_vfg_h_infinity.launch.py

# PH path + PID
ros2 launch qcar2_control_2 ph_vfg_control.launch.py

# Bezier path + PID
ros2 launch qcar2_control_2 bezier_vfg_control.launch.py

# Live GUI viewer (separate terminal, optional)
ros2 launch qcar2_control_2 dashboard_viewer.launch.py
```

---

## 7. Key Tuning Parameters by Stack

### H-infinity (`ph_vfg_h_infinity.launch.py`, current values)

```python
# vfg_guidance_ph_node
"k_e": 1.8
"lookahead_min_distance": 0.1
"lookahead_max_distance": 1.5
"lookahead_speed_min": 0.6
"lookahead_speed_max": 1.2

# h_inf_lateral_controller_node
"K_ff": 0.8
"rho_scale": 1.0
"output_gain": 0.6
"delta_max": 0.52
"steer_rate_limit_radps": 10.0
"speed_output_limit": 1.5
"steer_trim_rad": 0.0
"raw_encoder_counts": True   # QLabs default: velocity[0] = encoder counts/sec

# longitudinal_controller_node
"speed_max": 1.2
"speed_min": 0.6
"curvature_low": 0.1
"curvature_high": 0.3
"max_acceleration": 0.4
"max_deceleration": 0.6
"speed_smooth_alpha": 0.8
```

### FF lookahead (`vfg_guidance_ph_node`)

```
ff_lookahead_distance = ff_lookahead_base_m + ff_lookahead_tau_s × speed
                      = 0.1 + 0.16 × speed  [m]
```

### LPV H-infinity controller JSON — v1 / v2 / v3 / v3.1 history

The default controller JSON is `data/lpv_hinf_qcar2_v3.1.json`. Four iterations exist in the `data/` directory; selection is controlled by the `controller_json_path` parameter on `h_inf_lateral_controller_node`.

| JSON | Synthesis | γ range | ω_B range [rad/s] | Notes |
|---|---|---|---|---|
| `v1` | `siso_mixsyn_qcar2` (SISO mixsyn) | 1.61 – 4.92 | 2.19 – 4.11 | Initial mixsyn synthesis. γ > 1 at all vertices → spec not met. Kept for reference only. |
| `v2` | `lpv_hinf_qcar2_v2` (LPV, original W_e) | 0.71 – 0.87 | 3.0 – 5.0 | First LPV polytopic synthesis with original 1st-order shelving W_e. All vertices satisfy γ < 1 with comfortable margin, but K_LPV(0) ≈ 1.5e-5 – 4.1e-5 → near-zero DC gain causes steady-state lateral error to persist in step-curvature simulation (plant integrator cancelled by controller zero). |
| `v3` | `lpv_hinf_qcar2_v3` (LPV, W_e with LF-boost) | 0.74 – 0.95 | 3.0 – 5.875 | W_e augmented with a low-frequency boost shelving factor `(s + ω_z)/(s + ε)` with **ε = 1e-2, ω_z = 1.0 rad/s** (selected from parameter sweep). The added zero/pole pair raises DC demand by ~40 dB while preserving the HF asymptote (modulus margin floor). Result: K_LPV(0) ≈ 1.25e-3 at vertex 1 (~100× over v2), giving the controller near-integral character so it eliminates steady-state error without an external PI loop. Trade-off: γ moves closer to 1 (max 0.954 at vertex 6) and the common-Lyapunov margin tightens toward the feasibility boundary. |
| `v3.1` | `lpv_hinf_qcar2_v3.1` (LPV, v3 refined) | 0.68 – 0.90 | 3.0 – 5.875 | Current default. Same polytope (6 vertices over ρ ∈ [0, 2.25]) and ω_B schedule as v3, re-synthesized with refined weighting to enlarge the common-Lyapunov margin. γ_max drops from 0.954 → 0.899 (max vertex) while LF DC characteristics consistent with v3 are preserved, restoring headroom against the feasibility boundary that v3 was crowding. |

W_e structure (v3):

```
W_e(s, ρ) = (s/M_e + ω_B(ρ)) · (s + ω_z)
            ─────────────────────────────
              (s + ε) · (s + ω_B(ρ)·A_e)
```

where `A_e = 0.01` (DC accuracy template), `M_e = 2.0` (HF modulus margin template), `ω_B(ρ)` is sqrt-scheduled across the polytope. The shelving factor `(s + ω_z)/(s + ε)` is the only structural difference vs. v2.

> Background: the W_e extension and the ε / ω_z baseline selection (parameter sweeps, frequency-domain metrics, step-curvature simulation) are documented in the LPV H-infinity synthesis report (2026-05 week 2). v3.1 is the synthesis currently used in the simulator and on hardware; it shares v3's W_e structure but is re-synthesized for additional γ headroom.

---

## 8. Troubleshooting

- **`No path received`**: Check publisher node is running and topic name matches.
- **`STATUS_TF_FAIL`**: Check `base_frame` parameter and TF publisher.
- **Vehicle oscillates**: Lower `kp`/`kd` (PID) or `output_gain` (H-inf); reduce `speed_max`.
- **Late corner entry**: Increase `K_ff`; increase `ff_lookahead_base_m`.
- **Steady-state lateral error with H-inf**: Confirm `controller_json_path` is pointing at **v3 or v3.1** (not v2). v2 has K_LPV(0) ≈ 0 and cannot eliminate steady-state error without an external PI.
- **Dashboard viewer not shown**: `DISPLAY` not set — run `dashboard_viewer.launch.py` from a desktop session; CSV logging via `dashboard_logger` works headlessly regardless.
- **Wrong actual speed in viewer/CSV**: Check `raw_encoder_counts` parameter and verify `/qcar2_joint` `velocity[0]` units.
- **Lookahead curvature stuck at 0 / no curvature-based deceleration**: Resolved — see §9 (`std::pair` return ABI mismatch). If symptoms reappear after refactoring, do **not** change `PhPathRuntime::computeLookaheadMaxCurvature` back to a `std::pair<double,double>` return, and keep `ph_runtime` as an `OBJECT` library in CMakeLists.txt.

---

## 9. Resolved Issues

### 9.1 `std::pair<double,double>` return ABI mismatch in `PhPathRuntime::computeLookaheadMaxCurvature`

**Symptom**: Curvature-based deceleration in `longitudinal_controller_node` never engaged. The diagnostic logger reported:

```
[diag] |k_max| avg=0.0000 peak=0.0000 | d_look avg=0.000 m | target_v avg=0.5xx
```

`curvature_1pm` (instantaneous curvature at the closest point) was populated correctly with varying values, but `lookahead_distance_m` and `lookahead_max_curvature_1pm` on `/vfg/lateral_guidance` were **always exactly 0.0** (not NaN, not noise) across thousands of OK-status messages. Downstream `longitudinal_controller_node` consumed the 0 via `std::isfinite()` (which accepts 0 as valid) → `computeSpeedFromCurvature(0)` → `speed_max` always returned → never decelerated for curves.

**Diagnosis path** (in order of elimination):

1. *Build cache stale?* — Clean rebuild (`rm -rf build/qcar2_control_2 install/qcar2_control_2 && colcon build`) did not change the symptom.
2. *Multiple definition in archive?* — `nm -C build/qcar2_control_2/libph_runtime.a | grep -c computeLookaheadMaxCurvature` reported `2`, but inspection of the full output showed one was the function itself (`T`) and the other was the internal `static dbg_cnt` debug counter (`b`). False alarm.
3. *Schema mismatch on `LateralGuidance.msg`?* — Field order and types verified identical in src and install. Ruled out.
4. *Topic-name mismatch or extra publisher?* — `ros2 topic info --verbose` confirmed single publisher; subscribers consume the correct topic.
5. **Bit-level isnan probe** (`std::isnan()` checks at both ends of the function call) revealed the smoking gun:

   ```
   [PHRT-B]  ret.second=nan isnan=1 cnt=2301        ← inside ph_runtime: NaN is correctly produced
   [VFG-RAW] second=0.000000 isnan_second=0 cnt=2301 ← caller receives a real 0, not NaN
   ```

   Same call, same `cnt`, function body produced NaN bit-pattern, caller received zero bits.

**Root cause**: The function returned `std::pair<double,double>` (16 bytes, trivially-copyable). The x86_64 System V ABI allows two distinct returning conventions for such structs:

- **SSE classification** — first double in `XMM0`, second in `XMM1`.
- **Memory classification** — caller-allocated stack slot via hidden RDI pointer.

In this Isaac ROS humble podman container environment, the two translation units (`ph_runtime.cpp.o` and `vfg_guidance_ph.cpp.o`) classified the same return type **inconsistently** despite being compiled with the same compiler, headers, and flags. The callee wrote to one ABI's convention while the caller read from the other → caller observed uninitialized/zero bits. This is silent at link time (no error) and only manifests as wrong data at runtime.

**Fix**: Remove `std::pair` from the contract. Use out reference parameters instead — references are always passed as integer pointers (`RDI`/`RSI`), eliminating any classification ambiguity.

```cpp
// ph_runtime.hpp
void computeLookaheadMaxCurvature(
  double s_start,
  double lookahead_dist,
  int sample_count,
  double & out_actual_lookahead,
  double & out_max_curvature) const;
```

```cpp
// caller (vfg_guidance_ph.cpp)
double actual_lookahead = 0.0;
double lookahead_max_curvature = std::numeric_limits<double>::quiet_NaN();
runtime_.computeLookaheadMaxCurvature(
  closest.s, lookahead_distance, lookahead_sample_count_,
  actual_lookahead, lookahead_max_curvature);
lookahead_distance = actual_lookahead;
```

**Companion fixes applied at the same time**:

- **`ph_runtime` library type**: `STATIC` → `OBJECT` in `CMakeLists.txt`. STATIC archive (`.a`) incremental rebuild has been observed to leave stale `.o` references in some environments; OBJECT skips the archive step entirely and links `.o` directly into dependent executables. Also `target_include_directories` changed `PRIVATE` → `PUBLIC` so dependent targets inherit ph_runtime headers consistently.
- **`vfg_guidance_ph.cpp::publishStatus()`**: Now explicitly sets `curvature_1pm`, `lookahead_distance_m`, `lookahead_max_curvature_1pm`, and `ff_lookahead_curvature_1pm` to NaN on every error status path. Previously only `curvature_1pm` was set to NaN; the other three were left at the ROS message default (0.0), which would silently pass downstream `std::isfinite()` filters as "valid straight road" — a latent bug independent of the ABI issue.

**Verification after fix**:

```
[diag] |k_max| avg=0.5295 peak=2.6327 | d_look avg=0.185 m | target_v avg=0.466
[VFG] ... actual=0.197619 max=1.366894 curv=1.153727 cmd=0.400   ← speed cut from 0.6 to 0.4 in curve
[VFG] ... actual=0.123702 max=0.809823 curv=0.842576 cmd=0.400
```

Curvature-based deceleration engages correctly.

**Lessons**:

- Always verify NaN propagation at the bit level (`std::isnan()`), not via `printf("%f")`. `printf` formatting can lie under unusual circumstances; the bit check cannot.
- When return-by-value of a small POD/`std::pair` behaves inconsistently across translation units, prefer out reference parameters. The ABI is unambiguous (always pointer in integer register).
- Surgical probes (1–3 well-placed `fprintf` + `isnan` checks) beat wide-net logging when there is already a compressed symptom to anchor the hypothesis on.
