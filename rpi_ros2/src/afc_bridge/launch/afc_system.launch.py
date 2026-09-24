"""
afc_system.launch.py -- one-command bring-up of the whole onboard stack:

    uXRCE-DDS agent  (serial to Pixhawk/TELEM2)
    afc_bridge node  (FC setpoints -> RP2350; RP2350 telemetry -> ROS)
    rosbridge server (websocket :9090 for the browser dashboard)

  # everything, defaults (Tiny on uart3 = /dev/ttyAMA1, FC on serial0 = ttyAMA0):
  ros2 launch afc_bridge afc_system.launch.py

  # confirm the uart3 mapping on a new Pi (base fe201600 = uart3):
  dmesg | grep -iE 'ttyAMA'

  # turn a piece off if you're running it by hand:
  ros2 launch afc_bridge afc_system.launch.py start_agent:=false
  ros2 launch afc_bridge afc_system.launch.py start_rosbridge:=false

All FC I/O is uXRCE-DDS (no MAVROS). QGC/arming/missions ride the Doodle MAVLink
link straight to the FC. The dashboard connects to ws://<pi-ip>:<rosbridge_port>.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tiny_port = LaunchConfiguration("serial_port")
    agent_dev = LaunchConfiguration("agent_dev")
    agent_baud = LaunchConfiguration("agent_baud")
    start_agent = LaunchConfiguration("start_agent")
    start_rosbridge = LaunchConfiguration("start_rosbridge")
    rosbridge_port = LaunchConfiguration("rosbridge_port")

    # 1. uXRCE-DDS agent over the Pi PL011 (serial0 -> ttyAMA0) to Pixhawk TELEM2.
    agent = GroupAction(
        condition=IfCondition(start_agent),
        actions=[ExecuteProcess(
            cmd=["MicroXRCEAgent", "serial", "--dev", agent_dev, "-b", agent_baud],
            output="screen",
        )],
    )

    # 2. The bridge node (subscribes to FC topics, drives the tiny, keep-alive).
    bridge = Node(
        package="afc_bridge",
        executable="bridge_node",
        name="afc_bridge",
        output="screen",
        # topic defaults = /fmu/out setpoints; agent_port is added to the bridge's
        # forbidden list so it can never open the agent's (FC) UART.
        parameters=[{"serial_port": tiny_port, "agent_port": agent_dev}],
    )

    # 3. rosbridge websocket for the browser dashboard.
    rosbridge = GroupAction(
        condition=IfCondition(start_rosbridge),
        actions=[IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("rosbridge_server"),
                "launch", "rosbridge_websocket_launch.xml"])),
            launch_arguments={"port": rosbridge_port}.items(),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyAMA1"),   # Tiny, uart3
        DeclareLaunchArgument("agent_dev", default_value="/dev/serial0"),     # FC, ttyAMA0
        DeclareLaunchArgument("agent_baud", default_value="921600"),
        DeclareLaunchArgument("start_agent", default_value="false"),
        DeclareLaunchArgument("start_rosbridge", default_value="true"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        agent,
        bridge,
        rosbridge,
    ])
