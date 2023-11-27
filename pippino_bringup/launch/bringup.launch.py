import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

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

    display = LaunchConfiguration('display')

    # Launch pippino description (tf, 3d model, etc.)
    start_description = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_launch_dir, 'description.launch.py'))
    )
    # start_navigation = launch.actions.IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'navigation.launch.py')),
    #     launch_arguments = {'autostart': 'True'}.items()
    # )

    start_navigation_with_slam = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'bringup.launch.py')),
        launch_arguments = {
            'autostart': 'True',
            'use_composition': 'True',
            'use_respawn': 'False',
        }.items()
    )

    # start_slam = launch.actions.IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'slam.launch.py'))
    # )
    declare_display_enable = DeclareLaunchArgument(
        'display',
        default_value='true',
        description='Whether to launch display (rviz).')

    start_display = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(display_launch_dir, 'display.launch.py')),
        condition=IfCondition(display)
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
        # output={'both': 'log'},
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

    start_explorer_wanderer_server = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(explorer_wanderer_launch_dir, 'discoverer_launch.py'))
    )

    start_node_webserver = launch.actions.ExecuteProcess(
        cmd=['npm', 'start', '--prefix=~/ros-ui-react/example/']
    )

    return launch.LaunchDescription([
        # Start FastDDS server
        # start_fastdds_server
        declare_display_enable,
        ## start_description,
        ## launch.actions.TimerAction(period=8.0, actions=[start_navigation]),
        ##launch.actions.TimerAction(period=2.0, actions=[start_slam]),
        launch.actions.TimerAction(period=2.0, actions=[start_navigation_with_slam]),
        launch.actions.TimerAction(period=1.0, actions=[start_display]),
        rosbridge_server_node,
        ## teleop_twist_joy_node,
        aruco_detection_node,
        
        # Find yourself and map room
        start_explorer_wanderer_server,
        explorer_action_client_node,

        # Autodocking
        autodock_action_server_node,
        autodock_action_client_node,
        
        start_node_webserver,
        ## web_video_server_node
    ])

# pose at charger: Setting pose (1680945156.467487): x3.35 y=-0.55