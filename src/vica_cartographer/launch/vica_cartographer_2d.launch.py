import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('vica_cartographer')
    config_dir = os.path.join(pkg_share_dir, 'config')
    configuration_basename = 'vica_2d.lua'

    use_sim_time_arg = LaunchConfiguration('use_sim_time')
    resolution_arg = LaunchConfiguration('resolution')
    publish_period_arg = LaunchConfiguration('publish_period_sec')
    odom_topic_arg = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='실제 로봇 구동 시 false, rosbag 재생 매핑 시 true로 설정합니다.'
        ),

        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Occupancy Grid 지도 해상도입니다. 0.05는 5cm 격자를 의미합니다.'
        ),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='2D Occupancy Grid 지도를 /map으로 발행하는 주기입니다.'
        ),

        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
            description='Cartographer가 사용할 표준 EKF odometry 토픽입니다.'
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_arg,
            }],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', configuration_basename,
            ],
            remappings=[
                ('scan', '/scan'),

                ('odom', odom_topic_arg),
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time_arg,
            }],
            arguments=[
                '-resolution', resolution_arg,
                '-publish_period_sec', publish_period_arg,
            ],
        ),
    ])
