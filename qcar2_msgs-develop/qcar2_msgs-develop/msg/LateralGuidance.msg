std_msgs/Header header
uint32 curve_id

# Control output
float64 heading_error_rad

# Nearest point on curve
float64 u_star
geometry_msgs/Point closest_point

# Headings
float64 psi_vehicle_rad
float64 psi_path_tangent_rad
float64 psi_vfg_rad

# Errors
float64 cross_track_error_m
float64 curvature_1pm

# Lookahead curvature for longitudinal speed planning
float64 lookahead_distance_m
float64 lookahead_max_curvature_1pm

# Feedforward lookahead curvature (fixed distance ahead)
float64 ff_lookahead_curvature_1pm

# Weight scheduling / vector magnitudes
float64 w1_to_path
float64 w2_tangent
float64 v1_to_path_norm
float64 v_vfg_norm

# Status / diagnostics
uint8 status
string status_msg

uint8 STATUS_OK=0
uint8 STATUS_NO_PATH=1
uint8 STATUS_TF_FAIL=2
uint8 STATUS_INVALID_PATH=3
uint8 STATUS_NUMERIC_FAIL=4
uint8 STATUS_LIMITED=5
