uint8 NOT_INITIALIZED=0
uint8 INITIALIZING=1
uint8 TRACKING=2
uint8 DEGRADED=3
uint8 LOST=4
uint8 RELOCALIZING=5

std_msgs/Header header
uint8 state

uint32 tracked_features
float32 inlier_ratio
float32 mean_reprojection_error_px
float32 processing_time_ms

bool relocalization_event
uint32 map_id
string backend
