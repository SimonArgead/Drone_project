import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('nav_pkg')
    bridge_config = os.path.join(pkg_share, 'config', 'gazebo_ros2_bridge.yaml')

    return LaunchDescription([
        
        # GZ <-> ROS2 BRIDGE
        #Node(
        #    package='ros_gz_bridge',
        #    executable='parameter_bridge',
        #    name='gz_bridge_nav',
        #    parameters=[{'config_file': bridge_config}],
        #    output='screen'
        #),
        
        # Vision velocity node (RTAB-Map Odom -> TwistStamped)

        Node(
            package='nav_pkg',
            executable='autopilot',
            name='mid_level_autopilot',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {"pose_topic": "/zephyr/odom"},
                {"imu_topic": "/zephyr/imu/data"},
                {"mag_topic": "/zephyr/magnetometer"},
                {"baro_topic": "/zephyr/barometer"},
                {"gps_topic": "/zephyr/gps/fix"},
            ]
        ),

        Node(
            package='nav_pkg',
            executable='vision_twist_node',
            name='vision_twist_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Navigation controller
        Node(
            package='nav_pkg',
            executable='rtabmap_nav',
            name='rtabmap_nav',
            output='screen',
            parameters=[
                {'pose_topic': '/rtabmap/odom'},
                {'cmd_vel_topic': 'cmd_vel'},  # local name, remapped below
                {'use_sim_time': True},
                {'takeoff_duration_s': 3.0},
                {'vz_takeoff': 1.0},
                {'vx_forward': 1.0},
                {'vy_amplitude': 1.0},
                {'zig_period_s': 4.0},

                # Stereo camera topics (rectified)
                {'stereo_topic_left': '/left/image_rect'},
                {'stereo_topic_right': '/right/image_rect'},
                {'camera_info_topic_left': '/zephyr/camera_left/camera_info'},
                {'camera_info_topic_right': '/zephyr/camera_right/camera_info'}
            ],
            remappings=[
                ('cmd_vel', '/zephyr/mavros/setpoint_velocity/cmd_vel_unstamped'),
                ('/mission', '/autopilot/mission')
            ]
        ),

        Node(
            package='nav_pkg',
            executable='low_level_cnt',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )

    ])
