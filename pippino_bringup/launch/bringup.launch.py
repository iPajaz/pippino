import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='pippino_bringup').find('pippino_bringup')
    autostart = LaunchConfiguration('autostart')
    description_dir = launch_ros.substitutions.FindPackageShare(package='pippino_description').find('pippino_description') 
    description_launch_dir = os.path.join(description_dir, 'launch')
    navigation_dir = launch_ros.substitutions.FindPackageShare(package='pippino_navigation').find('pippino_navigation') 
    navigation_launch_dir = os.path.join(navigation_dir, 'launch')
    explorer_wanderer_launch_dir = launch_ros.substitutions.FindPackageShare(package='explorer_wanderer').find('explorer_wanderer')
    display_dir = launch_ros.substitutions.FindPackageShare(package='pippino_display').find('pippino_display')
    display_launch_dir = os.path.join(display_dir, 'launch')

    # Launch pippino description (tf, 3d model, etc.)
    start_description = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_launch_dir, 'description.launch.py'))
    )
    start_navigation = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'navigation.launch.py'))
    )
    start_slam = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'slam.launch.py'))
    )
    start_display = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(display_launch_dir, 'display.launch.py'))
    )

    rosbridge_server_node = launch_ros.actions.Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket'
    )

    web_video_server_node = launch_ros.actions.Node(
        package='web_video_server',
        executable='web_video_server',
    )

    teleop_twist_joy_node = launch_ros.actions.Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[os.path.join(pkg_share,'config/joystick.yaml'), {'use_sim_time': False}]
    )
    
    aruco_detection_node = launch_ros.actions.Node(
        package='aruco_detection',
        executable='aruco_detection_exe',
        name='aruco_detection_service',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'image_topic': '/D455/color/image_raw',
            'camera_frame': 'D455_color_optical_frame'
        }]
    )

    autodock_action_server_node = launch_ros.actions.Node(
        package='autodock_action_server',
        executable='autodock_action_server_exe',
        output='screen'
    )

    autodock_action_client_node = launch_ros.actions.Node(
        package='autodock_action_client',
        executable='autodock_action_client_exe',
        output='screen'
    )

    start_explorer_wanderer_server = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(explorer_wanderer_launch_dir, 'discoverer_launch.py'))
    )

    start_node_webserver = launch.actions.ExecuteProcess(
        cmd=['npm', 'start', '--prefix=~/ros-ui-react/example/']
    )

    return launch.LaunchDescription([
        ## start_description,
        launch.actions.TimerAction(period=2.0, actions=[start_navigation]),
        launch.actions.TimerAction(period=2.0, actions=[start_slam]),
        launch.actions.TimerAction(period=1.0, actions=[start_display]),
        ## rosbridge_server_node,
        ## teleop_twist_joy_node,
        start_explorer_wanderer_server,
        aruco_detection_node,
        autodock_action_server_node,
        autodock_action_client_node,
        start_node_webserver,
        ## web_video_server_node
    ])

# pose at charger: Setting pose (1680945156.467487): x3.35 y=-0.55