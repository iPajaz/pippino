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
    aruco_detection_pkg_prefix = get_package_share_directory('aruco_detection')
    rplidar_launch_pkg_prefix = get_package_share_directory('rplidar_ros2')
    description_dir = launch_ros.substitutions.FindPackageShare(package='pippino_description').find('pippino_description') 
    description_launch_dir = os.path.join(description_dir, 'launch')
    navigation_dir = launch_ros.substitutions.FindPackageShare(package='pippino_navigation').find('pippino_navigation') 
    navigation_launch_dir = os.path.join(navigation_dir, 'launch')

    explorer_wanderer_launch_dir = launch_ros.substitutions.FindPackageShare(package='explorer_wanderer').find('explorer_wanderer')


    navigation = LaunchConfiguration('navigation')
    microros = LaunchConfiguration('microros')
    lidar = LaunchConfiguration('lidar')
    start_description = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_launch_dir, 'description.launch.py'))
    )
    start_navigation = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'bringup_launch.py')),
        condition=IfCondition(navigation)
    )

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

    aruco_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [aruco_detection_pkg_prefix, '/launch/aruco_detection.launch.py']),
        launch_arguments={}.items()
    )

    micro_ros_agent_process = ExecuteProcess(
        cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent','serial', '--dev', '/dev/esp32'],
        output='screen',
        condition=IfCondition(microros)
    )

    start_lidar_process = ExecuteProcess(
        cmd=['uhubctl', '-l', '1-2.4.4', '-p', '1', '-a', 'on'],
        output='screen'
    )

    t265_docker_process = ExecuteProcess(
        cmd=['/home/michele/pippino_ws/src/docker_image_setup/run_daemon_realsense_t265.sh'],
        output='screen'
    )

    d455_docker_process = ExecuteProcess(
        cmd=['/home/michele/pippino_ws/src/docker_image_setup/run_daemon_realsense_d455.sh'],
        output='screen'
    )

    video_stream_controller_node = Node(
        package='video_stream_controller',
        executable='video_stream_controller_exe',
        name='video_stream_controller',
    )

    rosbridge_server_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        output='screen'
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

    start_explorer_wanderer_server = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(explorer_wanderer_launch_dir, 'discoverer_launch.py'))
    )

    autodock_action_server_node = launch_ros.actions.Node(
        package='autodock_action_server',
        executable='autodock_action_server_exe',
        output='screen'
        # output={'both': 'log'},
    )

    autodock_action_client_node = launch_ros.actions.Node(
        package='autodock_action_client',
        executable='autodock_action_client_exe',
        output='screen'
        # output={'both': 'log'},
    )

    explorer_action_client_node = launch_ros.actions.Node(
        package='explorer_action_client',
        executable='explorer_action_client_exe',
        output='screen'
        # output={'both': 'log'},
    )

    ld = LaunchDescription([
        DeclareLaunchArgument(name='navigation', default_value='True', description='Whether run the Navigation stack'),
        DeclareLaunchArgument(name='microros', default_value='True', description='Whether run the Microros agent node'),
        DeclareLaunchArgument(name='lidar', default_value='True', description='Whether run the Rplidar node'),

# Typical
        # Cameras
        d455_docker_process,
        t265_docker_process,

        pippino_odom_launch,
        start_lidar_process,
        video_stream_controller_node,
        aruco_detection_launch,
        rosbridge_server_node,
        web_video_server_node,
        teleop_twist_joy_node,

        launch.actions.TimerAction(period=5.0, actions=[rplidar_node]),
        launch.actions.TimerAction(period=6.0, actions=[micro_ros_agent_process]),
        launch.actions.TimerAction(period=6.0, actions=[start_description]),

        # Find yourself and map room
        launch.actions.TimerAction(period=3.0, actions=[start_explorer_wanderer_server]),
        launch.actions.TimerAction(period=8.0, actions=[explorer_action_client_node]),

        # Autodocking
        launch.actions.TimerAction(period=3.0, actions=[autodock_action_server_node]),
        launch.actions.TimerAction(period=8.0, actions=[autodock_action_client_node]),

        launch.actions.TimerAction(period=18.0, actions=[start_navigation]),

    ])

    
    return ld