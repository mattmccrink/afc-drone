#!/usr/bin/env python3
"""
dds_keepalive.py -- confirm/work around PX4 issue #25873.

When ROS 2 only SUBSCRIBES to PX4 (Payload rx == 0), uxrce_dds_client runs a
blocking 1-second ping every second, which stalls the FC's publish loop -> the
~1 Hz hiccups you see on /fmu/out/* topics. Publishing ANY /fmu/in/ topic makes
rx != 0, so the ping never fires.

This publishes an INERT OffboardControlMode (all fields false) at 5 Hz. It
commands NOTHING on its own -- offboard mode also requires arming + OFFBOARD
flight mode + actual setpoints, none of which this sends. It exists only to keep
the FC's rx alive.

Run (agent already up, px4_msgs built + sourced):
    source ~/afc-drone/rpi_ros2/install/setup.bash
    python3 dds_keepalive.py

Then watch the fix land:
    nsh> uxrce_dds_client status     # Payload rx: goes NONZERO; cycle max drops from ~1e6 us
    ros2 topic hz /fmu/out/vehicle_torque_setpoint   # rate stabilises, 1s max latency gone

Prereq: /fmu/in/offboard_control_mode must be in the FC's dds_topics.yaml
subscriptions (default set). If you pruned it out, this can't land and rx stays 0
-> then it's a firmware rebuild (and you'd do the 2-line #25873 fix instead).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from px4_msgs.msg import OffboardControlMode


class DdsKeepalive(Node):
    def __init__(self):
        super().__init__("dds_keepalive")
        # best-effort/volatile -- matches PX4's uXRCE-DDS QoS
        self.pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode",
            QoSPresetProfiles.SENSOR_DATA.value)
        self.create_timer(0.2, self._tick)      # 5 Hz is plenty to hold rx > 0
        self.get_logger().info(
            "keepalive: inert OffboardControlMode @5Hz -> /fmu/in/offboard_control_mode "
            "(workaround for PX4 #25873; commands nothing)")

    def _tick(self):
        m = OffboardControlMode()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)   # PX4 = microseconds
        # every field false = inert. If your px4_msgs version has a different
        # field set, `ros2 interface show px4_msgs/msg/OffboardControlMode`
        # and set them all False the same way.
        m.position = False
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        m.thrust_and_torque = False
        m.direct_actuator = False
        self.pub.publish(m)


def main():
    rclpy.init()
    node = DdsKeepalive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
