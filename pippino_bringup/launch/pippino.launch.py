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
    realsense_launch_pkg_prefix = get_package_share_directory('realsense2_camera')

    default_realsense_config_filename = LaunchConfiguration('default_realsense_config_filename')

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

    micro_ros_agent_process = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent','serial', '--dev', '/dev/esp32'],
        output='screen'
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [realsense_launch_pkg_prefix, '/launch/rs_launch.py']),
        launch_arguments = {'config_file': '\'/realsense_ws/src/d455.yaml\''}.items(),
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='default_realsense_config_filename', default_value='/realsense_ws/src/d455.yaml',
            description='Full path to the realsense config file to use'),

        rplidar_launch,
        micro_ros_agent_process,
        pippino_odom_launch,
        realsense_launch,
#        pippino_description_launch
    ])
