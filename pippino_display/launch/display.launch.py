import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='pippino_display').find('pippino_display')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/pippino_config.rviz')

    # teleop_node = launch_ros.actions.Node(
    # 	package='teleop_twist_keyboard',
    # 	executable='teleop_twist_keyboard',
    # 	name='teleop_node',
    # 	output='screen'
    # )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output={'both': 'log'},
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        rviz_node,
        # teleop_node
    ])