"""
afc_system.launch.py -- full stack for PX4 1.14+ (control allocation), dual-stack:

  MAVROS   (arm state, over fcu_url)            --> /mavros/state
  uXRCE-DDS agent  <--serial/udp-->  PX4        --> /fmu/out/vehicle_*_setpoint
  bridge   (torque+thrust -> CMD, state -> ARM) --> serial to the RP2350

  ros2 launch afc_bridge afc_system.launch.py \
       fcu_url:=udp://:14550@ \
       serial_port:=/dev/serial/by-id/usb-...RP2350... \
       start_agent:=true

Command demand is the PRE-allocation torque/thrust setpoint from PX4's rate
controller (the tiny does allocation). MAVROS carries only the arm state on
1.14+, since the old actuator-control message is gone with the mixer.

The uXRCE-DDS agent (MicroXRCEAgent) bridges PX4 uORB <-> ROS 2. Set
start_agent:=true to launch it here, or run it yourself and leave the default.
It must be installed and PX4's uxrce_dds_client pointed at it. px4_msgs must be
built in this workspace or the bridge's CMD input stays disabled.
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
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    tiny_port = LaunchConfiguration("serial_port")
    start_agent = LaunchConfiguration("start_agent")
    agent_port = LaunchConfiguration("agent_udp_port")

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "px4.launch"])
        ),
        launch_arguments={"fcu_url": fcu_url, "gcs_url": gcs_url}.items(),
    )

    # uXRCE-DDS agent. Adjust transport to match your PX4 uxrce_dds_client
    # config (udp4 -p 8888 is the common SITL/companion default).
    agent = GroupAction(
        condition=IfCondition(start_agent),
        actions=[ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", agent_port],
            output="screen",
        )],
    )

    bridge = Node(
        package="afc_bridge",
        executable="bridge_node",
        name="afc_bridge",
        output="screen",
        parameters=[{"serial_port": tiny_port}],   # topic defaults are the /fmu/out setpoints
    )

    return LaunchDescription([
        DeclareLaunchArgument("fcu_url", default_value="udp://:14550@"),
        DeclareLaunchArgument("gcs_url", default_value=""),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("start_agent", default_value="false"),
        DeclareLaunchArgument("agent_udp_port", default_value="8888"),
        agent,
        mavros,
        bridge,
    ])
