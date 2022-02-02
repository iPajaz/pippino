import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='pippino_description').find('pippino_description')
    default_model_path = os.path.join(pkg_share, 'src/description/pippino.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/pippino_config.rviz')
    slam = LaunchConfiguration('slam')
    nav2_dir = launch_ros.substitutions.FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    map_yaml_file = LaunchConfiguration('map')
    static_map_path = os.path.join(pkg_share, 'maps', 'upper_floor.yaml')
    nav2_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    nav2_bt_path = launch_ros.substitutions.FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    # behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    behavior_tree_xml_path = os.path.join(pkg_share, 'config', 'bt_navigate_to_pose.xml')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': False, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')])

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': False}]
    )

    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments = {'slam': slam,
                            'map': map_yaml_file,
                            'use_sim_time': 'false',
                            'params_file': nav2_params_path,
                            'default_bt_xml_filename': behavior_tree_xml_path,
                            'autostart': autostart
                            }.items())

    map_to_odom_tf_publisher_node = launch_ros.actions.Node(
        package = 'tf2_ros',
        executable = 'static_transform_publisher',
        arguments = ['0', '0', '0', '0', '0', '0', 'map', 'odom']
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
                                ('camera_info', '/camera/depth/camera_info'),
                                ('image_rect', '/camera/depth/image_rect_raw'),
                                ('points', '/camera/depth/color/points')]
                ),
            ],
            output='screen',
        )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='slam', default_value='False',
                                            description='Flag to enable SLAM'),
        launch.actions.DeclareLaunchArgument(name='map', default_value=static_map_path,
                                            description='Default map file full path'),
        launch.actions.DeclareLaunchArgument(name='params_file', default_value=nav2_params_path,
                                            description='Nav2 params file full path'),
        launch.actions.DeclareLaunchArgument(name='default_bt_xml_filename', default_value=behavior_tree_xml_path,
                                            description='Full path to the behavior tree xml file to use'),
        launch.actions.DeclareLaunchArgument(name='autostart', default_value='True',
                                            description='Automatically startup the nav2 stack'),
        point_cloud_generator,
        odom_to_base_link_tf_publisher_node,
        map_to_odom_tf_publisher_node,
        joint_state_publisher_node,
        robot_state_publisher_node,
        launch.actions.TimerAction(period=0.5, actions=[robot_localization_node]),
        launch.actions.TimerAction(period=1.0, actions=[start_ros2_navigation_cmd]),
        launch.actions.TimerAction(period=1.5, actions=[rviz_node]),
    ])