#!/usr/bin/env python3
"""
fake_mavros.py -- a stand-in for MAVROS, for bench-testing the bridge WITHOUT a
Pixhawk. It publishes the exact two topics the bridge subscribes to, so the node
runs unmodified and can't tell this from the real flight controller.

Run (no colcon needed -- it only uses system-installed mavros_msgs):

    source /opt/ros/$ROS_DISTRO/setup.bash
    python3 fake_mavros.py                 # gentle roll oscillation, auto-arms after 3 s
    python3 fake_mavros.py --hold          # fixed stick, no motion
    python3 fake_mavros.py --roll 0.7 --rate 30

Then watch the tiny's echo on /afc/ctrl_tlm:  source -> PRIMARY, valve[] moving,
armed -> true a few seconds in.

IMPORTANT: do NOT run this while real MAVROS is running -- two publishers on one
topic will fight. This is node-in-isolation. Ctrl-C the *state* stream (or pass
--drop-arm-after N) to watch the arm token go stale and the tiny disarm ~10 s later.

Why the defaults are what they are:
  * actuator_control at 30 Hz  -- faster than the bridge's cmd_stale_after_s (0.1 s),
    or 'source' flaps PRIMARY<->SBUS and looks broken.
  * disarmed for the first ~3 s -- the tiny refuses to honor arm until it has seen
    disarmed at least once; cold-arming leaves it in SAFE forever.
"""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from mavros_msgs.msg import State, ActuatorControl


class FakeMavros(Node):
    def __init__(self, args):
        super().__init__("fake_mavros")
        self.args = args
        self.t0 = time.monotonic()

        # SENSOR_DATA QoS matches how the bridge subscribes (best-effort).
        qos = QoSPresetProfiles.SENSOR_DATA.value
        self.pub_state = self.create_publisher(State, "/mavros/state", qos)
        self.pub_act = self.create_publisher(
            ActuatorControl, "/mavros/target_actuator_control", qos)

        self.create_timer(1.0 / args.rate, self._tick_actuator)   # fast: command
        self.create_timer(1.0 / 5.0, self._tick_state)            # slow: arm state

        self.get_logger().info(
            f"fake_mavros: actuator@{args.rate}Hz, state@5Hz, "
            f"disarmed for {args.arm_after}s then armed, "
            f"{'holding' if args.hold else f'roll +/-{args.roll}'}"
            + (f", dropping arm stream at t={args.drop_arm_after}s"
               if args.drop_arm_after else ""))

    def _elapsed(self) -> float:
        return time.monotonic() - self.t0

    def _tick_state(self):
        t = self._elapsed()
        # stop publishing state entirely after --drop-arm-after, to simulate an
        # FCU/MAVROS dropout: the bridge stops advancing the token -> tiny rides
        # the 10 s window then disarms.
        if self.args.drop_arm_after and t >= self.args.drop_arm_after:
            return
        m = State()
        m.header.stamp = self.get_clock().now().to_msg()
        m.connected = True
        m.armed = t >= self.args.arm_after       # disarmed first, then armed
        m.guided = True
        m.mode = "OFFBOARD"
        self.pub_state.publish(m)

    def _tick_actuator(self):
        t = self._elapsed()
        roll = 0.0 if self.args.hold else self.args.roll * math.sin(2 * math.pi * 0.25 * t)
        m = ActuatorControl()
        m.header.stamp = self.get_clock().now().to_msg()
        m.group_mix = 0                          # PX4 flight-control group
        # controls[0:4] = roll, pitch, yaw, thrust ; rest unused
        m.controls = [float(roll), 0.0, 0.0, float(self.args.thrust),
                      0.0, 0.0, 0.0, 0.0]
        self.pub_act.publish(m)


def main():
    ap = argparse.ArgumentParser(description="Fake MAVROS for bench-testing the bridge.")
    ap.add_argument("--rate", type=float, default=30.0, help="actuator_control Hz (keep >20)")
    ap.add_argument("--roll", type=float, default=0.4, help="roll oscillation amplitude [-1,1]")
    ap.add_argument("--thrust", type=float, default=0.3, help="constant thrust [0,1]")
    ap.add_argument("--hold", action="store_true", help="no motion, fixed sticks")
    ap.add_argument("--arm-after", type=float, default=3.0, help="seconds disarmed before arming")
    ap.add_argument("--drop-arm-after", type=float, default=0.0,
                    help="stop publishing /mavros/state at this t (0 = never); "
                         "use to test the stale-token disarm")
    args = ap.parse_args()

    rclpy.init()
    node = FakeMavros(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
