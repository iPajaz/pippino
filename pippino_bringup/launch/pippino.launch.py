import launch
from ament_index_python import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService
from launch_ros.actions import Node
import launch_ros
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('pippino_bringup')
    pippino_odom_pkg_prefix = get_package_share_directory('pippino_odom')
    # pippino_description_pkg_prefix = get_package_share_directory('pippino_description')
    rplidar_launch_pkg_prefix = get_package_share_directory('rplidar_ros2')
    # realsense_launch_pkg_prefix = get_package_share_directory('realsense2_camera')
    description_dir = launch_ros.substitutions.FindPackageShare(package='pippino_description').find('pippino_description') 
    description_launch_dir = os.path.join(description_dir, 'launch')
    navigation_dir = launch_ros.substitutions.FindPackageShare(package='pippino_navigation').find('pippino_navigation') 
    navigation_launch_dir = os.path.join(navigation_dir, 'launch')

    navigation = LaunchConfiguration('navigation')
    microros = LaunchConfiguration('microros')
    lidar = LaunchConfiguration('lidar')
    # default_realsense_config_filename = LaunchConfiguration('default_realsense_config_filename')

    # rplidar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [rplidar_launch_pkg_prefix, '/launch/rplidar_launch.py']),
    #     launch_arguments={'serial_port': '/dev/rplidar'}.items()
    # )
    start_description = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_launch_dir, 'description.launch.py'))
    )
    start_navigation = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'navigation.launch.py')),
        condition=IfCondition(navigation)
    )
    # batt_state_relay_node = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     name='batt_state_relay',
    #     parameters=['battery_level', 'batt_lvl']
    # )
    # batt_state_relay_process = ExecuteProcess(
    #     cmd=['ros2', 'run', 'topic_tools', 'relay', 'battery_level', 'batt_level']
    # )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{'channel_type': 'serial',
                     'serial_port': '/dev/rplidar', 
                     'serial_baudrate': 115200, 
                     'frame_id': 'laser',
                     'inverted': False, 
                     'angle_compensate': True,
                     'scan_mode': 'Sensitivity'}],
        output='screen',
        condition=IfCondition(lidar)
    )

    pippino_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [pippino_odom_pkg_prefix, '/launch/odom.launch.py']),
        launch_arguments={}.items()
    )

    # pippino_description_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [pippino_description_pkg_prefix, '/launch/description.launch.py']),
    #     launch_arguments={}.items()
    # )

    micro_ros_agent_process = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent','serial', '--dev', '/dev/esp32'],
        output='screen',
        condition=IfCondition(microros)
    )

    # realsense_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [realsense_launch_pkg_prefix, '/launch/rs_launch.py']),
    #     launch_arguments = {'config_file': '\'/realsense_ws/src/d455.yaml\''}.items(),
    # )

    video_stream_controller_node = Node(
        package='video_stream_controller',
        executable='video_stream_controller_exe',
        name='video_stream_controller',
    )

    rosbridge_server_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        # name='rosbridge_websocket'
    )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
    )

    teleop_twist_joy_node = launch_ros.actions.Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[os.path.join(pkg_share, 'config/joystick.yaml'), {'use_sim_time': False}]
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(name='navigation', default_value='True', description='Whether run the Navigation stack'),
        DeclareLaunchArgument(name='microros', default_value='True', description='Whether run the Microros agent node'),
        DeclareLaunchArgument(name='lidar', default_value='True', description='Whether run the Rplidar node'),
        ## DeclareLaunchArgument(name='default_realsense_config_filename', default_value='/realsense_ws/src/d455.yaml',
        ##     description='Full path to the realsense config file to use'),

        ## micro_ros_agent_process,
        ## launch.actions.TimerAction(period=2.0, actions=[pippino_odom_launch]),
        ## launch.actions.TimerAction(period=3.0, actions=[rplidar_launch]),
        
# Typical
        pippino_odom_launch,

        launch.actions.TimerAction(period=1.0, actions=[rplidar_node]),
        launch.actions.TimerAction(period=6.0, actions=[micro_ros_agent_process]),
        ## launch.actions.TimerAction(period=6.0, actions=[batt_state_relay_process]),
        launch.actions.TimerAction(period=6.0, actions=[start_description]),
        ## launch.actions.TimerAction(period=9.0, actions=[start_navigation]),
        video_stream_controller_node,
        rosbridge_server_node,
        web_video_server_node,
        teleop_twist_joy_node,
    ])

    
    return ld