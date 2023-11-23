#!/usr/bin/env python3

# (c) Michele De Marchi 2023

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image, CompressedImage, Joy, CameraInfo  # Image is the message type
from std_msgs.msg import Int32
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from pippino_service_msg.srv import PippinoVideoStream, PippinoActuators
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup

# Import Python libraries
import cv2  # OpenCV library
import ast
import numpy as np  # Import Numpy library
import time

class VideoStreamControllerConfig():
    d455_fast_profile = '640x360x30'
    d455_hires_profile = '1280x800x10'


class CustomLogger:
    def __init__(self, node, name):
        self.node = node
        self.logging_pub = node.create_publisher(String, 'pippino_ui_log', 1)
        self.logging_msg = String()
        self.logger = node.get_logger()
        self.name = name
        def get_logger():
            return self

        self.node.get_logger = get_logger

    def info(self, msg: str):
        self.logger.info(msg)
        self.logging_msg.data = f'{self.name}: {msg}'
        self.logging_pub.publish(self.logging_msg)

    def warning(self, msg: str):
        self.logger.warning(msg)
        self.logging_msg.data = f'{self.name} W: {msg}'
        self.logging_pub.publish(self.logging_msg)

    def error(self, msg: str):
        self.logger.error(msg)
        self.logging_msg.data = f'{self.name} E: {msg}'
        self.logging_pub.publish(self.logging_msg)



class VideoStreamController(Node):
    """
    Create an ArucoDetection class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('video_stream_controller')

        self.cfg = VideoStreamControllerConfig()

        CustomLogger(self, 'Video Stream Ctrl')

        self.declare_parameter("color_image_topic", "/D455/color/image_raw")
        self.declare_parameter("fisheye_image_topic", "/T265/fisheye1/image_raw")
        self.declare_parameter("camera_calibration_parameters_filename", "/pippino_ws/src/video_stream_controller/config/calibration_T265_fisheye1.yaml")

        self._curr_color_profile = None
        # Read parameters
        self.camera_calibration_parameters_filename = self.get_parameter(
                "camera_calibration_parameters_filename").get_parameter_value().string_value
        self.fisheye_image_topic = self.get_parameter("fisheye_image_topic").get_parameter_value().string_value
        self.color_image_topic = self.get_parameter("color_image_topic").get_parameter_value().string_value

        # Load the camera parameters from the saved file
        cv_file = cv2.FileStorage(
                self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
        self.mtx = cv_file.getNode('K').mat()
        self.dst = cv_file.getNode('D').mat()
        print(self.mtx, self.dst)
        cv_file.release()

        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        cam_param_cb_group = ReentrantCallbackGroup()
        
        self.d455_set_param_req = SetParameters.Request()
        self.d455_param_cli = self.create_client(SetParameters, '/D455/D455/set_parameters', callback_group=cam_param_cb_group)


        self.t265_set_param_req = SetParameters.Request()
        self.t265_param_cli = self.create_client(SetParameters, '/T265/T265/set_parameters', callback_group=cam_param_cb_group)

        # self.set_d455_color_camera_state(False)
        # self.set_t265_fisheye_camera_state(False)

        # self.joy_subscription = self.create_subscription(
        #     Joy, 
        #     '/joy', 
        #     self.joy_message_callback, 
        #     3)

        pub_sub_cb_group = ReentrantCallbackGroup()

        # self.compressed_image_msg = Image()
        # self.image_msg = Image()
        self.image_publisher = self.create_publisher(
            CompressedImage,
            '/strctl/image_raw/compressed',
            1,
            callback_group=pub_sub_cb_group
        )

        # self.base_image_publisher = self.create_publisher(
        #     Image,
        #     '/strctl/image_raw',
        #     1
        # )

        self.cam_info_publisher = self.create_publisher(
            CameraInfo,
            '/strctl/image_raw/camera_info',
            1,
            callback_group=pub_sub_cb_group
        )
        # self.compressed_image_publisher = self.create_publisher(
        #     Image,
        #     '/video',
        #     5
        # )

        self.color_subscription = self.create_subscription(
            Image, 
            self.color_image_topic,
            self.color_image_callback, 
            1,
            callback_group=pub_sub_cb_group
        )
        
        self.fisheye_subscription = self.create_subscription(
            Image, 
            self.fisheye_image_topic,
            self.fisheye_image_callback, 
            1,
            callback_group=pub_sub_cb_group
        )

        self.button_color_cam_counter = 0
        self.button_fisheye_cam_counter = 0
        self.color_enabled = False
        self.fisheye_enabled = False


        # self.pippino_actuators_cli = self.create_client(PippinoActuators, '/pippino_actuators')
        # while not self.pippino_actuators_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available.')
        video_stream_service_cb_group = ReentrantCallbackGroup()

        self.video_stream_service = self.create_service(PippinoVideoStream, 'pippino_video_stream', self.service_callback, callback_group=video_stream_service_cb_group)
        self.get_logger().info('Pippino video stream controller service up and running.')
        # self.actuators_relay_service = self.create_service(PippinoActuators, 'pippino_actuators_relay', self.pippino_actuators_relay_cb)


        # self._loop_rate = self.create_rate(1)  # 1 Hz



    def set_t265_fisheye_camera_state(self, enabled):
        if self.fisheye_enabled != enabled:
            self.t265_set_param_req.parameters = [Parameter(name='enable_fisheye1', value=enabled).to_parameter_msg(), Parameter(name='enable_fisheye2', value=enabled).to_parameter_msg()]
            if not self.t265_param_cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().info('T265 service not available.')
                return False

            t265_set_param_future = self.t265_param_cli.call_async(self.t265_set_param_req)
            self.executor.spin_until_future_complete(t265_set_param_future, timeout_sec=10.0)

            if t265_set_param_future.result() is None or not t265_set_param_future.result().results[0].successful:
                return False
            self.fisheye_enabled = enabled

        return True

    def set_d455_color_camera_state(self, enabled, profile=None):
        if profile is not None and self._curr_color_profile != profile:
            # set profile and disable color
            self.get_logger().info(f"Trying to set color camera profile...")
            if not self.d455_param_cli.wait_for_service(timeout_sec=10.0):
                self.get_logger().info('D455 service not available.')
                return False

            self.d455_set_param_req.parameters = [Parameter(name='enable_color', value=False).to_parameter_msg(), Parameter(name='rgb_camera.profile', value=profile).to_parameter_msg()]
            d455_set_param_future = self.d455_param_cli.call_async(self.d455_set_param_req)

            self.executor.spin_until_future_complete(d455_set_param_future, timeout_sec=10.0)

            if d455_set_param_future.result() is None or not d455_set_param_future.result().results[0].successful:
                return False

            self._curr_color_profile = profile
            self.color_enabled = False

            if enabled:
                # d455_set_param_future.add_done_callback(self.set_d455_color_camera_on)
                # self.set_d455_color_camera_on()
                self.get_logger().info(f"Trying to {'enable' if enabled else 'disable'} color camera...")
                self.d455_set_param_req.parameters = [Parameter(name='enable_color', value=True).to_parameter_msg()]
                d455_set_param_future = self.d455_param_cli.call_async(self.d455_set_param_req)
        
                self.executor.spin_until_future_complete(d455_set_param_future, timeout_sec=10.0)
    
                if d455_set_param_future.result() is None or not d455_set_param_future.result().results[0].successful:
                    return False

                self.color_enabled = True


        elif self.color_enabled != enabled:
            self.get_logger().info(f"Trying to {'enable' if enabled else 'disable'} color camera...")
            # if not self.d455_param_cli.wait_for_service(timeout_sec=10.0):
            #     self.get_logger().info('D455 service not available.')
            #     return False

            self.d455_set_param_req.parameters = [Parameter(name='enable_color', value=enabled).to_parameter_msg()]
            d455_set_param_future = self.d455_param_cli.call_async(self.d455_set_param_req)

            self.executor.spin_until_future_complete(d455_set_param_future, timeout_sec=10.0)
            
            if d455_set_param_future.result() is None or not d455_set_param_future.result().results[0].successful:
                return False

            self.color_enabled = enabled
        # breakpoint()
        return True

    def color_image_callback(self, data):
        # print("publishing")
        # breakpoint()
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(current_frame)
        # self.compressed_image_msg.data = compressed_img
        # self.image_msg = data
        # self.get_logger().info("publishing")
        compressed_image_msg.header = data.header
        compressed_image_msg.format = "rgb8; jpeg compressed rgb8"
        self.image_publisher.publish(compressed_image_msg)

        # msg = CameraInfo()
        # msg.header = data.header
        # msg.height = 800
        # msg.width = 1280
        # compressed_image_msg.format = "bgr8; jpeg compressed rgb8"

        # self.cam_info_publisher.publish(msg)


    # def color_image_callback(self, data):
    #     self.image_publisher.publish(data)
        
        # msg = Image()
        # msg.height=800
        # msg.width=1280
        # msg.encoding="rgb8"
        # msg.step=3840
        # msg.header = data.header
        # msg.data = [0] * (800 *3840)
        # self.base_image_publisher.publish(msg)


    def fisheye_image_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="mono8")
        undistorted_image = cv2.fisheye.undistortImage(current_frame, self.mtx, self.dst, Knew=self.mtx, new_size=(1000, 1000))

        # map_1, map_2 = cv2.fisheye.initUndistortRectifyMap(self.mtx, self.dst, np.eye(3), self.mtx, (1000, 1000), cv2.CV_32F)
        # undistorted_image = cv2.remap(current_frame, map_1, map_2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(undistorted_image)
        compressed_image_msg.header = data.header
        compressed_image_msg.format = "mono8; jpeg compressed mono8"

        self.image_publisher.publish(compressed_image_msg)

        # image_msg = self.bridge.cv2_to_imgmsg(undistorted_image, encoding='mono8')
        # self.image_publisher.publish(image_msg)
    
    # def pippino_actuators_relay_cb(self, request, response):
    #     self.get_logger().warning("trying to set vacuum")
    #     self.req = PippinoActuators.Request()
    #     self.req.request_type = 1
    #     self.req.bool_vacuum_enable = True        
    #     self.pippino_actuators_cli.call_async(self.req)
    #     response.success = True
    #     return response

    def get_d455_rgb_profile(self, hires):
            if hires:
                return self.cfg.d455_hires_profile
            else:
                return self.cfg.d455_fast_profile

    def service_callback(self, request, response):
        # self.get_logger().info(f"setting cameras {request.request_type}")
        if request.request_type == 0:
            if self.set_d455_color_camera_state(request.d455_enable):
                self.get_logger().info(f"Color camera {'enabled' if request.d455_enable else 'disabled'}.")
            else:
                response.success = False
                self.get_logger().error("Failed to set camera params.")
                return response
        elif request.request_type == 1:
            if self.set_d455_color_camera_state(request.d455_enable, self.get_d455_rgb_profile(request.d455_hires)):
                self.get_logger().info(f"Color camera {'enabled' if request.d455_enable else 'disabled'}.")
            else:
                response.success = False
                self.get_logger().error("Failed to set camera params.")
                return response
        elif request.request_type == 2:
            if self.set_t265_fisheye_camera_state(request.t265_enable):
                self.get_logger().info(f"Fisheye camera {'enabled' if request.t265_enable else 'disabled'}.")
            else:
                response.success = False
                self.get_logger().error("Failed to set camera params.")
                return response
        elif request.request_type == 3:
            if self.set_d455_color_camera_state(request.d455_enable):
                self.get_logger().info(f"Color camera {'enabled' if request.d455_enable else 'disabled'}.")
            else:
                response.success = False
                self.get_logger().error("Failed to set camera params.")
                return response
            if self.set_t265_fisheye_camera_state(request.t265_enable):
                self.get_logger().info(f"Fisheye camera {'enabled' if request.t265_enable else 'disabled'}.")
            else:
                response.success = False
                self.get_logger().error("Failed to set camera params.")
                return response
        elif request.request_type == 4:
            if self.set_d455_color_camera_state(request.d455_enable, self.get_d455_rgb_profile(request.d455_hires)):
                self.get_logger().info(f"Color camera {'enabled' if request.d455_enable else 'disabled'}.")
            else:
                response.success = False
                self.get_logger().error("Failed to set camera params.")
                return response
            self.set_t265_fisheye_camera_state(request.t265_enable)

        response.success=True
        return response


        # # self.get_logger().warning(f"Fubar {data}")
        # if data.buttons[0] == 1:
        # #     self.button_color_cam_counter += 1
        # #     self.button_fisheye_cam_counter = 0
        # #     print(self.button_color_cam_counter)
        # # elif data.buttons[1] == 1:
        # #     self.button_color_cam_counter = 0
        # #     self.button_fisheye_cam_counter += 1
        # #     print(self.button_fisheye_cam_counter)
        # # if self.button_color_cam_counter == 20:
        #     self.set_t265_fisheye_camera_state(False)
        #     self.set_d455_color_camera_state(True, profile=self.cfg.d455_fast_profile)
        #     # self.button_color_cam_counter = 0
        #     self.color_enabled = True
        #     self.fisheye_enabled = False
        # elif data.buttons[1] == 1:
        # # elif self.button_fisheye_cam_counter == 20:
        #     self.set_t265_fisheye_camera_state(True)
        #     self.set_d455_color_camera_state(False)
        #     # self.button_fisheye_cam_counter = 0
        #     self.fisheye_enabled = True
        #     self.color_enabled = False
        # elif data.buttons[2] == 1:
        #     self.set_t265_fisheye_camera_state(False)
        #     self.set_d455_color_camera_state(False)
        # elif len(data.buttons) > 8 and data.buttons[8] == 1:
        #     self.set_t265_fisheye_camera_state(False)
        #     self.set_d455_color_camera_state(True, profile=self.cfg.d455_hires_profile)


def main(args=None):
    rclpy.init(args=args)

    video_stream_controller = VideoStreamController()

    rclpy.spin(video_stream_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
