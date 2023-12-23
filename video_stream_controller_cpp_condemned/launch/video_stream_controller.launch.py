from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    d455_fast_profile = LaunchConfiguration('d455_fast_profile')
    d455_hires_profile = LaunchConfiguration('d455_hires_profile')
    color_image_topic = LaunchConfiguration('color_image_topic')
    fisheye_image_topic = LaunchConfiguration('fisheye_image_topic')
    fisheye_cam_calib_param_file = LaunchConfiguration('fisheye_cam_calib_param_file')

    declare_d455_fast_profile_cmd = DeclareLaunchArgument(
        'd455_fast_profile',
        default_value='640x360x30',
        description='The color camera high frame rate profile for user manual navigation.')
    declare_d455_hires_profile_cmd = DeclareLaunchArgument(
        'd455_hires_profile',
        default_value='848x480x10',
        description='The color camera high-res profile for accurate image processing (e.g. aruco search).')
    declare_color_image_topic_cmd = DeclareLaunchArgument(
        'color_image_topic',
        default_value='/D455/color/image_raw',
        description='The color camera raw image topic.')
    declare_fisheye_image_topic_cmd = DeclareLaunchArgument(
        'fisheye_image_topic',
        default_value='/T265/fisheye1/image_raw',
        description='The fisheye camera raw image topic.')
    declare_fisheye_cam_calib_param_file_cmd = DeclareLaunchArgument(
        'fisheye_cam_calib_param_file',
        default_value='/home/michele/pippino_ws/src/video_stream_controller_cpp/config/calibration_T265_fisheye1.yaml',
        description='The file name for the camera calibration parameters.')

    return LaunchDescription([
        declare_d455_fast_profile_cmd,
        declare_d455_hires_profile_cmd,
        declare_color_image_topic_cmd,
        declare_fisheye_image_topic_cmd,
        declare_fisheye_cam_calib_param_file_cmd,
        Node(
            package='video_stream_controller_cpp',
            executable='video_stream_controller',
            name='video_stream_controller2',
            output='screen',
            parameters=[{
                'd455_fast_profile': d455_fast_profile,
                'd455_hires_profile': d455_hires_profile,
                'color_image_topic': color_image_topic,
                'fisheye_image_topic': fisheye_image_topic,
                'fisheye_cam_calib_param_file': fisheye_cam_calib_param_file,
            }]
        ),
    ])