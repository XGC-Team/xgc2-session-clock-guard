std_msgs/Header header

uint8 EVENT_NEW_EPOCH=0
uint8 EVENT_LOCKED=1
uint8 EVENT_DEGRADED=2
uint8 EVENT_LOST=3

uint8 STATE_INITIALIZING=0
uint8 STATE_LOCKED=1
uint8 STATE_DEGRADED=2
uint8 STATE_LOST=3

string session_id
string session_contract_sha256
string policy_sha256
string run_mode
string session_time_authority
string clock_mapping
uint64 sequence
uint8 event
uint8 state
uint64 epoch
uint64 monotonic_stamp_ns
uint64 session_stamp_ns
uint64 authority_age_ns
uint64 last_gazebo_clock_stamp_ns
int64 gazebo_clock_skew_ns
int64 station_wall_error_ns
int64 station_wall_step_ns
float64 gazebo_real_time_factor
string reason
