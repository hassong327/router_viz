# Single 3D object localization result
string class_name           # Object class name
int32 class_id              # Class ID
float32 confidence          # Detection confidence (0.0 ~ 1.0)
geometry_msgs/Point position  # 3D position in map frame (meters)
float32 distance            # Distance from base_link origin (meters)
