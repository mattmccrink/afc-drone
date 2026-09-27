# Sensor telemetry from the RP2350 valve node (SENSOR_TLM, frame type 0x82).
# Dual timebase: header.stamp is the Pi's ROS receipt time; node_stamp_ms is the
# tiny's own millis() at frame build. Record both -- receipt time alone smears
# the 25 Hz stream with USB/scheduling jitter and breaks tight event correlation.
std_msgs/Header header

uint32 node_stamp_ms        # RP2350 millis() when the frame was built

float32[6] p_up             # upstream pressure, mbar
float32[6] p_lo             # throat pressure, mbar
float32[6] t_die            # sensor die temperature, deg C
float32[6] mdot             # per-valve mass flow, g/s
float32 mdot_total          # summed over VALID valves, g/s
bool[6] valid               # per-valve validity
uint8[6] why                # per-venturi validity reason (tiny VentHealth); 255 = unknown (old fw)
uint8 VH_OK = 0
uint8 VH_UNMAPPED = 1
uint8 VH_NOPROM = 2
uint8 VH_NODATA = 3
uint8 VH_STALE = 4
uint8 VH_FROZEN = 5
uint8 VH_LOSSY = 6
uint8 VH_RANGE = 7
uint8 VH_DPNEG = 8
uint8 VH_RECOVER = 9
uint8 VH_INJECTED = 10
