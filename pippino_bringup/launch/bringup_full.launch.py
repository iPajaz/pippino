import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
import yaml

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='pippino_bringup').find('pippino_bringup')
    autostart = LaunchConfiguration('autostart')
    description_dir = launch_ros.substitutions.FindPackageShare(package='pippino_description').find('pippino_description') 
    description_launch_dir = os.path.join(description_dir, 'launch')
    navigation_dir = launch_ros.substitutions.FindPackageShare(package='pippino_navigation').find('pippino_navigation') 
    navigation_launch_dir = os.path.join(navigation_dir, 'launch')
    display_dir = launch_ros.substitutions.FindPackageShare(package='pippino_display').find('pippino_display')
    display_launch_dir = os.path.join(display_dir, 'launch')

    # Launch pippino description (tf, 3d model, etc.)
    start_description = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_launch_dir, 'description.launch.py'))
    )
    start_navigation = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'navigation.launch.py'))
    )
    start_display = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(display_launch_dir, 'display.launch.py'))
    )

    pippino_commander_node = launch_ros.actions.Node(
        package='pippino_commander',
        executable='pippino_commander_exe',
        name='pippino_commander'
    )

    rosbridge_server_node = launch_ros.actions.Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket'
    )

    web_video_server_node = launch_ros.actions.Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
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
        name='aruco_detection',
        output='screen'
    )

    autodock_action_server_node = launch_ros.actions.Node(
        package='autodock_action_server',
        executable='autodock_action_server_exe',
        name='autodock_action_server',
        output='screen'
    )    

    autodock_action_client_node = launch_ros.actions.Node(
        package='autodock_action_client',
        executable='autodock_action_client_exe',
        name='autodock_action_client',
        output='screen'
    )    

    start_node_webserver = launch.actions.ExecuteProcess(
        cmd=['npm', 'start', '--prefix=~/ros-ui-react/example/']
    )

    with open(os.path.join(pkg_share, 'config/mqtt_settings.yaml'), 'r') as stream:
        mqtt_settings = yaml.safe_load(stream)

    wake_up_pippino = launch.actions.ExecuteProcess(
        cmd=['mosquitto_pub', '-h', mqtt_settings['broker'], '-p', '1883', '-t', 'pippino/power', '-u', mqtt_settings['auth']['username'], '-P', mqtt_settings['auth']['password'], '-m', '1']
    )

    return launch.LaunchDescription([
        wake_up_pippino,
        # start_description,
        launch.actions.TimerAction(period=20.0, actions=[start_navigation]),
        # launch.actions.TimerAction(period=1.0, actions=[start_display]),
        launch.actions.TimerAction(period=20.0, actions=[
            web_video_server_node,
            rosbridge_server_node,
            teleop_twist_joy_node,
            aruco_detection_node,
            autodock_action_server_node,
            autodock_action_client_node,
            start_node_webserver,
            pippino_commander_node
        ])
    ])

# pose at charger: Setting pose (1680945156.467487): x3.35 y=-0.55