"""Start Rokae dual-arm drag and publish mapped joints for Marvin ServoJ teleop."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("rokea_drag_bridge")
    dds_xml = os.path.join(share, "dds", "fastdds_unicast_nimo.xml")
    # Allow override: NIMO_DDS_PEER not baked into XML; XML peer is Jetson default.
    peer_xml = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", dds_xml)

    return LaunchDescription(
        [
            SetEnvironmentVariable(
                name="RMW_IMPLEMENTATION", value="rmw_fastrtps_cpp"
            ),
            SetEnvironmentVariable(
                name="FASTRTPS_DEFAULT_PROFILES_FILE", value=peer_xml
            ),
            DeclareLaunchArgument("rate_hz", default_value="50.0"),
            DeclareLaunchArgument("local_ip", default_value="192.168.2.100"),
            DeclareLaunchArgument("ip_a", default_value="192.168.2.160"),
            DeclareLaunchArgument("ip_b", default_value="192.168.2.161"),
            DeclareLaunchArgument(
                "left_topic", default_value="arm/left/target_joints"
            ),
            DeclareLaunchArgument(
                "right_topic", default_value="arm/right/target_joints"
            ),
            DeclareLaunchArgument(
                "status_topic", default_value="rokea/drag_bridge/status"
            ),
            DeclareLaunchArgument(
                "enable_topic", default_value="rokea/drag_bridge/enable"
            ),
            Node(
                package="rokea_drag_bridge",
                executable="rokea_drag_bridge_node",
                name="rokea_drag_bridge",
                output="screen",
                parameters=[
                    {
                        "rate_hz": ParameterValue(
                            LaunchConfiguration("rate_hz"), value_type=float
                        ),
                        "local_ip": LaunchConfiguration("local_ip"),
                        "ip_a": LaunchConfiguration("ip_a"),
                        "ip_b": LaunchConfiguration("ip_b"),
                        "left_topic": LaunchConfiguration("left_topic"),
                        "right_topic": LaunchConfiguration("right_topic"),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "enable_topic": LaunchConfiguration("enable_topic"),
                        # Rokea → Marvin: flip joints 2/4/6 (1-based)
                        "sign_flip": [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0],
                    }
                ],
            ),
        ]
    )
