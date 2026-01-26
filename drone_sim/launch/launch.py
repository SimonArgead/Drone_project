import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # 1) Paths
    pkg_share = get_package_share_directory('drone_sim')

    world = os.path.join(pkg_share, 'worlds', 'my_world.sdf')

    # 2) Start Gazebo Harmonic
    gz = ExecuteProcess(
        cmd=[
            "gz", "sim",
            "-r",
            "-v", "4",
            world
        ],
        output="screen"
    )

    # 3) ros_gz_bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": os.path.join(pkg_share, "config", "gz_bridge.yaml")
        }],
        output="screen"
    )

    return LaunchDescription([
        gz,
        bridge,
    ])
