import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='pippino_description').find('pippino_description')
    default_model_path = os.path.join(pkg_share, 'src/description/pippino.urdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': False, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': ['power_probe_joint_state']}]
    )
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': False}],
        remappings=[('/odometry/filtered', '/odom')]
    )
    map_to_odom_tf_publisher_node = launch_ros.actions.Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    odom_frame_to_base_link_tf_publisher_node = launch_ros.actions.Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'odom_frame', 'base_link']
    )
    odom_to_base_link_tf_publisher_node = launch_ros.actions.Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )
    point_cloud_generator = launch_ros.actions.ComposableNodeContainer(
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
                                ('camera_info', '/D455/depth/camera_info'),
                                ('image_rect', '/D455/depth/image_rect_raw'),
                                ('points', '/D455/depth/color/points')]
                ),
            ],
            output='screen',
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        ## point_cloud_generator,
        odom_to_base_link_tf_publisher_node,
        map_to_odom_tf_publisher_node,
        odom_frame_to_base_link_tf_publisher_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        launch.actions.TimerAction(period=0.5, actions=[robot_localization_node])
    ])
