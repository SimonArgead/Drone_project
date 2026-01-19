import os
from ament_index_python.packages import get_package_share_directory
import xacro

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # --- 1) Paths ---
    pkg_share = get_package_share_directory('drone_sim')

    world = os.path.join(pkg_share, 'worlds', 'my_world.world')
    xacro_file = os.path.join(pkg_share, 'urdf', 'zephyr_delta_wing.xacro')

    # --- 2) Parse Xacro → URDF (TF only) ---
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_desc = doc.toxml()

    # --- 3) Start Gazebo Harmonic (server + GUI) ---
    gz = ExecuteProcess(
        cmd=[
            "gz", "sim",
            "-r",          # start server immediately
            "-v", "4",     # verbose
            world          # absolute world path
        ],
        output="screen"
    )

    # --- 4) ros_gz_bridge ---
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{
            "config_file": os.path.join(pkg_share, "config", "gz_bridge.yaml")
        }],
        output="screen"
    )

    # --- 5) Robot State Publisher ---
    state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_desc},
            {"use_sim_time": True}
        ],
        output="screen"
    )

    # --- 6) Spawn SDF model (AFTER server is ready) ---
    model_path = os.path.join(
        pkg_share,
        'models',
        'zephyr_delta_wing',
        'zephyr_delta_wing.sdf'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', model_path,
            '-name', 'zephyr_delta_wing',
            '-x', '0', '-y', '0', '-z', '31'
        ],
        output='screen'
    )

    # Delay spawn by 2 seconds so server is ready
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[spawn]
    )

    return LaunchDescription([
        gz,
        clock_bridge,
        state_pub,
        delayed_spawn,
    ])
