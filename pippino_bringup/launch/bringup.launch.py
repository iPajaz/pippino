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

    

    return launch.LaunchDescription([
        start_description,
        launch.actions.TimerAction(period=1.0, actions=[start_navigation]),
        launch.actions.TimerAction(period=1.0, actions=[start_display]),
    ])