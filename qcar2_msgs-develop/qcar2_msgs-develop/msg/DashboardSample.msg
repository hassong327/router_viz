std_msgs/Header header

float64 elapsed_sec
float64 distance_travelled_m

float64 vehicle_x_map
float64 vehicle_y_map

float64 target_speed_mps
float64 actual_speed_mps
float64 steering_rad
float64 cross_track_error_m

# Topic readiness flags (true after first message of each kind seen by logger)
bool tf_ok
bool motor_cmd_ok
bool guidance_ok

bool completed
