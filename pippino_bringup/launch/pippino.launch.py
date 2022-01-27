import launch
from ament_index_python import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
    pippino_odom_pkg_prefix = get_package_share_directory('pippino_odom')
    pippino_description_pkg_prefix = get_package_share_directory('pippino_description')
    rplidar_launch_pkg_prefix = get_package_share_directory('rplidar_ros')

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [rplidar_launch_pkg_prefix, '/launch/rplidar.launch.py']),
        launch_arguments={}.items()
    )
    
    pippino_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pippino_odom_pkg_prefix, '/launch/odom.launch.py']),
        launch_arguments={}.items()
    )

    pippino_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pippino_description_pkg_prefix, '/launch/description.launch.py']),
        launch_arguments={}.items()
    )

    # micro_ros_agent_node = launch_ros.actions.Node(
    #     package='micro_ros_agent',
    #     executable='micro_ros_agent',
    #     name='micro_ros_agent_node',
    #     output='screen',
    #     parameters=['serial',
    #         '--dev', '/dev/esp32'
    #     ]
    # )

    micro_ros_agent_process = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent','serial', '--dev', '/dev/esp32'],
        output='screen'
    )


    return launch.LaunchDescription([
        rplidar_launch,
        micro_ros_agent_process,
        pippino_odom_launch,
        pippino_description_launch
    ])