"""
afc_system.launch.py -- one-command bring-up of the whole onboard stack:

    uXRCE-DDS agent  (serial to Pixhawk/TELEM2)
    afc_bridge node  (FC setpoints -> RP2350; RP2350 telemetry -> ROS)
    rosbridge server (websocket :9090 for the browser dashboard)

  # everything, defaults:
  ros2 launch afc_bridge afc_system.launch.py \
       serial_port:=/dev/serial/by-id/usb-...RP2350... \
       agent_dev:=/dev/serial/by-id/usb-...TELEM2-FTDI...

  # turn a piece off if you're running it by hand:
  ros2 launch afc_bridge afc_system.launch.py start_agent:=false
  ros2 launch afc_bridge afc_system.launch.py start_rosbridge:=false

All FC I/O is uXRCE-DDS (no MAVROS). QGC/arming/missions live on a separate
telemetry radio. The dashboard connects to ws://<pi-ip>:<rosbridge_port>.
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

    # 1. uXRCE-DDS agent over serial to the Pixhawk (FTDI on TELEM2).
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
        parameters=[{"serial_port": tiny_port}],   # topic defaults = /fmu/out setpoints
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
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("agent_dev", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("agent_baud", default_value="921600"),
        DeclareLaunchArgument("start_agent", default_value="true"),
        DeclareLaunchArgument("start_rosbridge", default_value="true"),
        DeclareLaunchArgument("rosbridge_port", default_value="9090"),
        agent,
        bridge,
        rosbridge,
    ])
