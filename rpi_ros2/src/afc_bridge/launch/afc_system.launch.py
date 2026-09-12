"""
afc_system.launch.py -- full stack, all uXRCE-DDS (no MAVROS).

  uXRCE-DDS agent  <--serial-->  PX4 (USB/TELEM2)  -->  /fmu/out/vehicle_*
  bridge  (torque+thrust -> CMD, vehicle_status -> ARM)  -->  serial to RP2350

  ros2 launch afc_bridge afc_system.launch.py \
       serial_port:=/dev/serial/by-id/usb-...RP2350... \
       agent_dev:=/dev/serial/by-id/usb-...PX4... \
       start_agent:=true

QGC / arming / mission planning live on a SEPARATE telemetry radio (MAVLink),
not this link -- so the Pi<->FC connection carries only DDS telemetry, one link,
one middleware. ARM comes from vehicle_status.arming_state (strict == ARMED).

px4_msgs must be built in this workspace or the bridge inputs stay disabled.
Firmware prereq: /fmu/out/vehicle_torque_setpoint + /fmu/out/vehicle_thrust_setpoint
added to dds_topics.yaml (vehicle_status is published by default).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tiny_port = LaunchConfiguration("serial_port")
    start_agent = LaunchConfiguration("start_agent")
    agent_dev = LaunchConfiguration("agent_dev")
    agent_baud = LaunchConfiguration("agent_baud")

    # uXRCE-DDS agent over serial to the Pixhawk (USB CDC or TELEM2 UART).
    agent = GroupAction(
        condition=IfCondition(start_agent),
        actions=[ExecuteProcess(
            cmd=["MicroXRCEAgent", "serial", "--dev", agent_dev, "-b", agent_baud],
            output="screen",
        )],
    )

    bridge = Node(
        package="afc_bridge",
        executable="bridge_node",
        name="afc_bridge",
        output="screen",
        parameters=[{"serial_port": tiny_port}],   # topic defaults are the /fmu/out topics
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("start_agent", default_value="false"),
        DeclareLaunchArgument("agent_dev", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("agent_baud", default_value="921600"),
        agent,
        bridge,
    ])
