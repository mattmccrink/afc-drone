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

# ---- fault-tree / mode (from the tiny's own state, via CTRL_TLM) ----
uint8 MODE_DISARMED       = 0
uint8 MODE_PRIMARY_ARMED  = 1
uint8 MODE_SBUS_REVERSION = 2
uint8 MODE_SAFE_HOLD      = 3   # armed, no live source (pre-termination debounce)
uint8 MODE_TERMINATED     = 4
uint8 mode                     # derived operating mode (single source of truth = tiny)
string mode_str                # human-readable mode, for echo/dashboards

bool terminated                # latched flight-termination (total comms loss)
bool elig_fc                   # FC arm path eligible (seen disarmed once)
bool elig_sbus                 # SBUS arm path eligible (seen disarmed once)
bool sbus_sw                   # current SBUS arm-switch state
float32 desat_rp               # roll/pitch desaturation scale (1.0 = not clipping)
float32 desat_yaw              # yaw desaturation scale (1.0 = not clipping)
