"""Launch the bridge node, optionally recording a correlation rosbag.

  ros2 launch afc_bridge bridge.launch.py                 # node only
  ros2 launch afc_bridge bridge.launch.py record:=true    # + rosbag
  ros2 launch afc_bridge bridge.launch.py serial_port:=/dev/ttyACM1 record:=true

Recording is done by an external `ros2 bag record` process rather than a
rosbag2_py writer inside the node -- that keeps the node free of rosbag2 API
drift across distros, and bags exactly the four topics needed for post-flight
cause/effect: what the node received from MAVROS (command + arm intent) and what
the tiny reported back.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    torque_topic = LaunchConfiguration("torque_topic")
    thrust_topic = LaunchConfiguration("thrust_topic")
    status_topic = LaunchConfiguration("status_topic")
    ctrl_topic = LaunchConfiguration("ctrl_topic")
    sensor_topic = LaunchConfiguration("sensor_topic")
    record = LaunchConfiguration("record")
    bag_uri = LaunchConfiguration("bag_uri")

    node = Node(
        package="afc_bridge",
        executable="bridge_node",
        name="afc_bridge",
        output="screen",
        parameters=[{
            "serial_port": serial_port,
            "torque_topic": torque_topic,
            "thrust_topic": thrust_topic,
            "status_topic": status_topic,
            "ctrl_topic": ctrl_topic,
            "sensor_topic": sensor_topic,
        }],
    )

    # Bag the command demand (torque+thrust), the arm input, and both telemetry
    # streams -- everything needed for post-flight cause/effect.
    bag = GroupAction(
        condition=IfCondition(record),
        actions=[ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", bag_uri,
                 torque_topic, thrust_topic, status_topic, ctrl_topic, sensor_topic],
            output="screen",
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("torque_topic",
                              default_value="/fmu/out/vehicle_torque_setpoint"),
        DeclareLaunchArgument("thrust_topic",
                              default_value="/fmu/out/vehicle_thrust_setpoint"),
        DeclareLaunchArgument("status_topic", default_value="/fmu/out/vehicle_status"),
        DeclareLaunchArgument("ctrl_topic", default_value="/afc/ctrl_tlm"),
        DeclareLaunchArgument("sensor_topic", default_value="/afc/sensor_tlm"),
        DeclareLaunchArgument("record", default_value="false"),
        DeclareLaunchArgument("bag_uri", default_value="afc_flight"),
        node,
        bag,
    ])
