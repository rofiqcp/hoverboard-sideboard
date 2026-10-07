from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("sideboard_p2_fusion")
    p2_yaml = os.path.join(share, "config", "p2_fusion.yaml")
    local_yaml = os.path.join(share, "config", "ekf_local.yaml")
    global_yaml = os.path.join(share, "config", "ekf_global.yaml")
    navsat_yaml = os.path.join(share, "config", "navsat_transform.yaml")

    wheel_topic = LaunchConfiguration("wheel_topic")
    steering_topic = LaunchConfiguration("steering_topic")
    lio_topic = LaunchConfiguration("lio_topic")
    gps_topic = LaunchConfiguration("gps_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    enable_serial = LaunchConfiguration("enable_serial_bridge")
    enable_local_ekf = LaunchConfiguration("enable_local_ekf")
    enable_gnss = LaunchConfiguration("enable_gnss")

    p2 = Node(
        package="sideboard_p2_fusion",
        executable="p2_fusion_node",
        name="sideboard_p2_fusion",
        output="screen",
        parameters=[
            p2_yaml,
            {
                "wheel_odom_topic": wheel_topic,
                "steering_topic": steering_topic,
                "lio_odom_topic": lio_topic,
            },
        ],
    )

    serial_bridge = Node(
        package="sideboard_p2_fusion",
        executable="serial_aiding_bridge",
        name="sideboard_serial_aiding_bridge",
        output="screen",
        parameters=[p2_yaml],
        condition=IfCondition(enable_serial),
    )

    local_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        output="screen",
        parameters=[local_yaml],
        remappings=[
            ("/Odometry", lio_topic),
            ("odometry/filtered", "/odometry/local"),
        ],
        condition=IfCondition(enable_local_ekf),
    )

    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[navsat_yaml],
        remappings=[
            ("gps/fix", gps_topic),
            ("imu", imu_topic),
            ("odometry/filtered", "/odometry/local"),
            ("odometry/gps", "/odometry/gps"),
        ],
        condition=IfCondition(enable_gnss),
    )

    global_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        output="screen",
        parameters=[global_yaml],
        remappings=[("odometry/filtered", "/odometry/global")],
        condition=IfCondition(enable_gnss),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "wheel_topic", default_value="/wheel/odometry_raw"
            ),
            DeclareLaunchArgument(
                "steering_topic", default_value="/steering/angle_rad"
            ),
            DeclareLaunchArgument("lio_topic", default_value="/Odometry"),
            DeclareLaunchArgument("gps_topic", default_value="/fix"),
            DeclareLaunchArgument(
                "imu_topic", default_value="/imu/enu_absolute"
            ),
            DeclareLaunchArgument(
                "enable_serial_bridge", default_value="false"
            ),
            DeclareLaunchArgument("enable_local_ekf", default_value="true"),
            DeclareLaunchArgument("enable_gnss", default_value="false"),
            p2,
            serial_bridge,
            local_ekf,
            navsat,
            global_ekf,
        ]
    )
