# Control telemetry from the RP2350 valve node (CTRL_TLM, frame type 0x81).
# header.stamp is the Pi's ROS receipt time. This frame carries NO node clock
# (only ValveNodeSensor does), so cross-stream correlation uses the sensor
# frame's node_stamp_ms plus the offset it establishes.
std_msgs/Header header

# active command source
uint8 SOURCE_PRIMARY = 0
uint8 SOURCE_SBUS    = 1
uint8 SOURCE_SAFE    = 2
uint8 source

bool armed

# compressor mode
uint8 COMP_STOPPED  = 0
uint8 COMP_TRACK    = 1
uint8 COMP_DIRECT   = 2
uint8 COMP_FALLBACK = 3
uint8 comp_mode

bool flow_fallback          # 30k fail-toward-airflow fallback active
uint16 rpm_target           # commanded compressor rpm
uint8 n_valid               # count of valves feeding the flow sum

int16[6] valve              # aero-intent valve positions
uint16[12] servo_us         # resolved servo commands (us)
