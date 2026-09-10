"""
afc_system.launch.py -- bring up the whole flight stack: MAVROS (PX4) + the
AFC bridge node, in one command.

  ros2 launch afc_bridge afc_system.launch.py \
       fcu_url:=/dev/serial/by-id/usb-...-Pixhawk:921600 \
       serial_port:=/dev/serial/by-id/usb-...-Tiny2350

Two SEPARATE serial devices:
  * fcu_url     -> the Pixhawk (MAVROS talks to this)
  * serial_port -> the RP2350 tiny (the bridge talks to this)
Do not confuse them. If both enumerate as /dev/ttyACM*, the numbers can swap on
reboot -- use /dev/serial/by-id/ symlinks (stable per device) for both.

MAVROS takes a few seconds to connect to the FCU before /mavros/state and
/mavros/target_actuator_control start publishing; the bridge tolerates their
absence and simply doesn't forward until they appear, so no ordering is needed.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    gcs_url = LaunchConfiguration("gcs_url")
    tiny_port = LaunchConfiguration("serial_port")
    cmd_topic = LaunchConfiguration("cmd_topic")

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "px4.launch"])
        ),
        launch_arguments={"fcu_url": fcu_url, "gcs_url": gcs_url}.items(),
    )

    bridge = Node(
        package="afc_bridge",
        executable="bridge_node",
        name="afc_bridge",
        output="screen",
        parameters=[{
            "serial_port": tiny_port,
            "cmd_topic": cmd_topic,
        }],
    )

    return LaunchDescription([
        # fcu_url: serial to the Pixhawk. USB companion link is typically
        # 921600; a telemetry radio is 57600. Prefer a by-id symlink.
        DeclareLaunchArgument("fcu_url", default_value="/dev/ttyACM1:921600"),
        DeclareLaunchArgument("gcs_url", default_value=""),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM1"),
        DeclareLaunchArgument("cmd_topic",
                              default_value="/mavros/target_actuator_control"),
        mavros,
        bridge,
    ])
