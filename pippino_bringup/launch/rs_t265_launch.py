# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.


# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_multi_camera_launch.py camera_name1:=D400 device_type2:=l5. device_type1:=d4..

"""Launch realsense2_camera node."""
import copy
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch

local_parameters = [{'name': 'camera_name1', 'default': 'T265', 'description': 'camera unique name'},
                    {'name': 'device_type1', 'default': 't265', 'description': 'choose device by type'},
                    {'name': 'enable_fisheye11', 'default': 'false', 'description': 'topic for T265 wheel odometry'},
                    {'name': 'enable_fisheye21', 'default': 'false', 'description': 'topic for T265 wheel odometry'},
                   ]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params
    

def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) + 
        [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_launch.py']),
            launch_arguments=set_configurable_parameters(params1).items(),
        ),
        ## # dummy static transformation from camera1 to camera2
        # launch_ros.actions.Node(
        #     package = "video_stream_controller_cpp",
        #     executable = "video_stream_controller",
        #     name = "video_stream_controller",
        # ),
    ])
