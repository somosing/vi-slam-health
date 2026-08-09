uint8 INITIALIZING=0
uint8 HEALTHY=1
uint8 DEGRADED=2
uint8 LOST=3
uint8 RELOCALIZING=4
uint8 RECOVERED=5
uint8 RESETTING=6

std_msgs/Header header
uint8 state

float32 score
bool pose_valid
bool relocalization_requested

float32 feature_quality
float32 inlier_quality
float32 reprojection_quality
float32 sensor_quality
float32 realtime_quality

string[] reasons
