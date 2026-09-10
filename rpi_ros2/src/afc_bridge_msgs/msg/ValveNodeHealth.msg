# Node/link health, published at 1 Hz on /afc/health.
# The bench + wifi-viz replacement for the tiny's serial 'health' console: with
# the USB port owned by the node, this is how you see link state remotely.
std_msgs/Header header

bool serial_connected        # is the USB port open right now
float32 ctrl_hz              # measured CTRL_TLM receive rate (nominal ~25)
float32 sensor_hz            # measured SENSOR_TLM receive rate (nominal ~25)
uint32 crc_errors           # cumulative dropped frames -- flat 0 = clean link,
                            # climbing = baud/desync/noise
uint32 frames_ok            # cumulative good frames decoded

# readback of the TX path, straight from the tiny's most recent CTRL_TLM:
uint8 SOURCE_PRIMARY = 0
uint8 SOURCE_SBUS    = 1
uint8 SOURCE_SAFE    = 2
uint8 SOURCE_NONE    = 255
uint8 last_source           # PRIMARY only if our CMD is arriving fresh + valid
bool last_armed             # tiny's arm state -> proves the arm-token path

# node-side liveness of the two MAVROS inputs:
bool cmd_fresh              # node is currently forwarding CMD (control input fresh)
bool arm_fresh             # arm token is advancing (/mavros/state fresh)
uint32 arm_counter         # current monotonic arm-token value
int32 tlm_offset_ms        # est. (node millis - ROS ms); jumps = clock issue
