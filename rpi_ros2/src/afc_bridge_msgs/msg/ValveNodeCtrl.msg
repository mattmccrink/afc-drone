# Control telemetry from the RP2350 valve node (CTRL_TLM, frame type 0x81).
# header.stamp is the Pi's ROS receipt time; node_stamp_ms (end of message) is the
# tiny's own millis() at frame build, the same clock as ValveNodeSensor and
# ValveNodeComp, so control, sensor and compressor frames align in bags without
# USB/scheduling jitter.
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

bool surf_engaged           # surfaces ACTIVE: allocated with AFC (switch on, live source, not terminated)
bool surf_switch            # surface switch position (SBUS / console force), regardless of mode
int16[4] surf               # surface commands [-1000,1000], 0 = configured center
                            # order: L wing, R wing, L canard, R canard

# ---- air-delivery severity (tiny air_state.h, 2026-09-27). Reporting only. ----
# Ranks how well air delivery is CONFIRMED: lost ESC telemetry with healthy
# venturis < lost venturis < both / venturis showing no flow while running.
uint8 AIR_OK       = 0
uint8 AIR_ADVISORY = 1
uint8 AIR_CAUTION  = 2
uint8 AIR_WARNING  = 3
uint8 air_sev               # debounced on the tiny
uint8 AIR_C_TLM_LOST     = 1    # Teensy: no ESC telemetry, open loop (err -11)
uint8 AIR_C_TLM_EMU      = 2    # Teensy: emulation build (err -12)
uint8 AIR_C_VENT_PARTIAL = 4    # some venturis invalid
uint8 AIR_C_VENT_LOST    = 8    # no valid venturis
uint8 AIR_C_FLOW_LOW     = 16   # venturis healthy but flow implausibly low while running
uint8 AIR_C_SENSOR_STALE = 32   # sensor frame not updating
uint8 AIR_C_TEENSY_LINK  = 64   # no Teensy replies while armed
uint8 air_causes            # bitmask of AIR_C_* behind the level on display (held while air_sev is lit, 0 at OK)

# ---- RC link + build configuration (CTRL_TLM flags2, 2026-09-27) ----
bool flags2_valid           # false on older firmware (fields below then meaningless)
bool sbus_lost              # SBUS receiver reports failsafe / frame-lost
bool sbus_ok                # clean SBUS frames arriving
bool bench_build            # tiny built with BENCH_HOOKS=1 -- NOT a flight build
bool cal_flash              # calibration loaded from flash (false = compiled defaults)
bool sim_sensors            # tiny built with SIMULATED sensors/servos (USE_REAL_I2C=0)
bool sim_sbus               # tiny built with SIMULATED SBUS (USE_REAL_SBUS=0)
bool sim_primary            # tiny built with SIMULATED primary input (USE_REAL_PRIMARY=0)

# ---- node clock (CTRL_TLM ext5, 2026-09-29) ----
uint32 node_stamp_ms        # RP2350 millis() when the frame was built; 0 = older firmware
