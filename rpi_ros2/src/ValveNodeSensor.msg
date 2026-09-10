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
