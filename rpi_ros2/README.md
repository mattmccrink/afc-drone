# Raspberry Pi / ROS2 Hub

Supervisor + telemetry bridge (NOT an in-loop relay): feeds the RP2350 over USB
serial, relays the Pixhawk arm token, and publishes valve/pressure/temp/mass-flow
telemetry. The RP2350 commands the Teensy directly; the Pi is out of the fast loop.

**Status:** not started (design only). First tasks: ROS2 node owning the serial +
magic-resync reader, demux by frame `type`, subscribe to Pixhawk command topics,
relay the arm token (1 Hz heartbeat + on-change, never latch stale). Reuse the
`host_tools` framing as the starting point.

Colcon `build/`, `install/`, `log/` are gitignored.
