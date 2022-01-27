
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    default_rviz = os.path.join(get_package_share_directory('depth_image_proc'),
                                'launch', '/home/michele/pippino_ws/src/pippino_description/rviz/point_cloud.rviz')
    return LaunchDescription([
        # install realsense from https://github.com/intel/ros2_intel_realsense
        # launch_ros.actions.Node(
        #     package='realsense_ros2_camera', node_executable='realsense_ros2_camera',
        #     output='screen'),

        # launch plugin through rclcpp_components container
        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzNode',
                    name='point_cloud_xyz_node',
                    remappings=[
                                ('camera_info', '/camera/depth/camera_info'),
                                ('image_rect', '/camera/depth/image_rect_raw'),
                                ('points', '/camera/depth/color/points')]
                ),
            ],
            output='screen',
        ),

        # rviz
        launch_ros.actions.Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['--display-config', default_rviz]),
    ])
