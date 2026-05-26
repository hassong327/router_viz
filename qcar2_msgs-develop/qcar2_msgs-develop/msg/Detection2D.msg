# Single 2D detection result from YOLO
string class_name        # Object class name (e.g., "stop_sign", "traffic_light")
int32 class_id           # Class ID from YOLO model
float32 confidence       # Detection confidence (0.0 ~ 1.0)
float32 center_x         # Bounding box center x (pixels)
float32 center_y         # Bounding box center y (pixels)
float32 width            # Bounding box width (pixels)
float32 height           # Bounding box height (pixels)

# Extended fields for depth-aware localization pipeline
float32 depth            # Raw depth value (meters)
float32 depth_sample     # Sampled/filtered depth used for localization (meters)
int32 depth_sample_u     # Depth sample pixel u
int32 depth_sample_v     # Depth sample pixel v
float32 position_x       # 3D position x in camera frame (meters)
float32 position_y       # 3D position y in camera frame (meters)
float32 position_z       # 3D position z in camera frame (meters)
