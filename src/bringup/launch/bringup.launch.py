from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [PathJoinSubstitution([FindPackageShare("raspimouse"), "launch", "raspimouse.launch.py"])]
                )
            ),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
            ),
            Node(
                package="aquestalkpi_ros",
                executable="aquestalkpi_ros",
            ),
            Node(
                package="nanodet_ros",
                executable="nanodet_ros",
            ),
            Node(
                package="controller",
                executable="controller",
            ),
        ]
    )
