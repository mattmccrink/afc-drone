# Measured compressor telemetry from the Motor Teensy (COMP_TLM, 0x83),
# relayed by the RP2350. Measured quantities only; commanded rpm is in
# ValveNodeCtrl. header.stamp = Pi receipt time; node_stamp_ms = tiny clock.
std_msgs/Header header

float32 volt          # ESC pack voltage (V)
float32 amp           # ESC current (A)
int32   rpm           # measured mechanical rpm
int16   temp_c        # ESC temperature (deg C)
int8    err           # ESCCMD err code (0 = ok, -10 = TLM_LOST)
bool    tlm_ok        # Teensy had fresh ESC telemetry (err==0 & not stale)
uint32  node_stamp_ms # tiny millis() at emit, for cross-stream correlation
bool    thermal_suspect  # ESC hot (>=90 C) AND rpm sagging >10% below target for >=3 s
                         # (ESC likely self-derating). Advisory -- pilot-actionable.
