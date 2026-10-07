from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_driver = LaunchConfiguration("enable_livox_driver")
    enable_lio = LaunchConfiguration("enable_lio")
    enable_serial = LaunchConfiguration("enable_serial_bridge")
    enable_gnss = LaunchConfiguration("enable_gnss")
    livox_launch_file = LaunchConfiguration("livox_launch_file")
    lio_config_file = LaunchConfiguration("lio_config_file")
    lio_rviz = LaunchConfiguration("lio_rviz")

    livox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("livox_ros_driver2"),
                    "launch_ROS2",
                    livox_launch_file,
                ]
            )
        ),
        condition=IfCondition(enable_driver),
    )

    lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("fast_lio_sam"), "launch", "mapping.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": lio_config_file,
            "rviz": lio_rviz,
        }.items(),
        condition=IfCondition(enable_lio),
    )

    p2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("sideboard_p2_fusion"),
                    "launch",
                    "p2_fusion.launch.py",
                ]
            )
        ),
        launch_arguments={
            "lio_topic": "/Odometry",
            "enable_serial_bridge": enable_serial,
            "enable_local_ekf": "true",
            "enable_gnss": enable_gnss,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_livox_driver", default_value="true"
            ),
            DeclareLaunchArgument("enable_lio", default_value="true"),
            DeclareLaunchArgument(
                "enable_serial_bridge", default_value="false"
            ),
            DeclareLaunchArgument("enable_gnss", default_value="false"),
            DeclareLaunchArgument(
                "livox_launch_file", default_value="msg_MID360s_launch.py"
            ),
            DeclareLaunchArgument(
                "lio_config_file", default_value="mid360.yaml"
            ),
            DeclareLaunchArgument("lio_rviz", default_value="false"),
            livox,
            lio,
            p2,
        ]
    )
