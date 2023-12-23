import launch
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='pippino_navigation').find('pippino_navigation')
    slam = LaunchConfiguration('slam')
    nav2_dir = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    map_yaml_file = LaunchConfiguration('map')
    static_map_path = os.path.join(pkg_share, 'maps', 'upper_floor.yaml')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params_humble.yaml')
    nav2_bt_path = launch_ros.substitutions.FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    # behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    behavior_tree_xml_path = os.path.join(pkg_share, 'config', 'bt_navigate_to_pose.xml')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments = {#'slam': slam,
                            #'map': map_yaml_file,
                            'use_sim_time': 'False',
                            'params_file': nav2_params_path,
                            'default_bt_xml_filename': behavior_tree_xml_path,
                            'autostart': autostart
                            }.items())

    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument(name='slam', default_value='False',
        #                                     description='Flag to enable SLAM'),
        # launch.actions.DeclareLaunchArgument(name='map', default_value=static_map_path,
        #                                     description='Default map file full path'),
        launch.actions.DeclareLaunchArgument(name='params_file', default_value=nav2_params_path,
                                            description='Nav2 params file full path'),
        launch.actions.DeclareLaunchArgument(name='default_bt_xml_filename', default_value=behavior_tree_xml_path,
                                            description='Full path to the behavior tree xml file to use'),
        launch.actions.DeclareLaunchArgument(name='autostart', default_value='True',
                                            description='Automatically startup the nav2 stack'),
        start_ros2_navigation_cmd,
        # launch.actions.TimerAction(period=1.5, actions=[rviz_node]),
    ])