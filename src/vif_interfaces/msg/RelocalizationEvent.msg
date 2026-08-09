uint8 REQUESTED=0
uint8 STARTED=1
uint8 SUCCEEDED=2
uint8 FAILED=3
uint8 RESET_REQUESTED=4
uint8 NEW_MAP_REQUESTED=5

std_msgs/Header header
uint8 event
uint32 attempt_id
float32 health_score
float32 elapsed_sec
string detail
