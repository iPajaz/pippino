import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    odom_node = Node(
            # the name of the executable is set in CMakeLists.txt, towards the end of
            # the file, in add_executable(...) and the directives following it
            package='pippino_odom',
            executable='pippino_odom',
            output='screen',
            parameters=[{
                'angular_scale_positive':1.0,
                'angular_scale_negative':1.0,
                'linear_scale_positive':1.0,
                'linear_scale_negative':1.0,
                'wheelbase_m_': 0.28,
            }],
            # remappings=[('/pippino/odom', '/odom')]
        )
    
    return LaunchDescription([
        odom_node,
    ])
