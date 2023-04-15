#!/usr/bin/env python3

# Publishes a coordinate transformation between an ArUco marker and a camera
# Adapted from:
# - Addison Sears-Collins
# - https://automaticaddison.com

import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image, CompressedImage, Joy  # Image is the message type
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

# Import Python libraries
import cv2  # OpenCV library
import ast
import numpy as np  # Import Numpy library
import time


class VideoStreamController(Node):
    """
    Create an ArucoDetection class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('aruco_detection_service')

        self.declare_parameter("color_image_topic", "/D455/color/image_raw")
        self.declare_parameter("fisheye_image_topic", "/T265/fisheye1/image_raw")
        self.declare_parameter("camera_calibration_parameters_filename", "/pippino_ws/src/video_stream_controller/config/calibration_T265_fisheye1.yaml")


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

        self.set_param_req = SetParameters.Request()
        self.d455_param_cli = self.create_client(SetParameters, '/D455/D455/set_parameters')
        while not self.d455_param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.set_param_req1 = SetParameters.Request()
        self.t265_param_cli = self.create_client(SetParameters, '/T265/T265/set_parameters')
        while not self.t265_param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # self.set_d455_color_camera_state(False)
        # self.set_t265_fisheye_camera_state(False)

        self.joy_subscription = self.create_subscription(
            Joy, 
            '/joy', 
            self.joy_message_callback, 
            3)

        # self.compressed_image_msg = Image()
        # self.image_msg = Image()
        self.image_publisher = self.create_publisher(
            Image,
            '/video',
            1
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
            1
        )
        
        self.fisheye_subscription = self.create_subscription(
            Image, 
            self.fisheye_image_topic,
            self.fisheye_image_callback, 
            1
        )

        self.button_color_cam_counter = 0
        self.button_fisheye_cam_counter = 0
        self.color_enabled = False
        self.fisheye_enabled = False


    def set_t265_fisheye_camera_state(self, enabled):
        self.get_logger().warning("Trying to set fisheye parameters")
        self.set_param_req1.parameters = [Parameter(name='enable_fisheye1', value=enabled).to_parameter_msg(), Parameter(name='enable_fisheye2', value=enabled).to_parameter_msg()]
        self.get_logger().warning("Calling service...")
        self.future = self.t265_param_cli.call_async(self.set_param_req1)
        self.get_logger().warning("Done...")
        # time.sleep(1)

    def set_d455_color_camera_state(self, enabled):
        self.get_logger().warning(f"Setting D455 color cam state to {enabled}")
        self.set_param_req.parameters = [Parameter(name='enable_color', value=enabled).to_parameter_msg()]
        self.future = self.d455_param_cli.call_async(self.set_param_req)
        # time.sleep(1)

    # def color_image_callback(self, data):
    #     # print("publishing")
    #     # breakpoint()
    #     current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    #     compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(current_frame)
    #     # self.compressed_image_msg.data = compressed_img
    #     # self.image_msg = data
    #     self.get_logger().info("publishing")
    #     self.image_publisher.publish(compressed_image_msg)

    def color_image_callback(self, data):
        self.image_publisher.publish(data)

    def fisheye_image_callback(self, data):
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="mono8")
        undistorted_image = cv2.fisheye.undistortImage(current_frame, self.mtx, self.dst, Knew=self.mtx, new_size=(1000, 1000))

        # map_1, map_2 = cv2.fisheye.initUndistortRectifyMap(self.mtx, self.dst, np.eye(3), self.mtx, (1000, 1000), cv2.CV_32F)
        # undistorted_image = cv2.remap(current_frame, map_1, map_2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        # compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(undistorted_image)
        # self.image_publisher.publish(compressed_image_msg)

        image_msg = self.bridge.cv2_to_imgmsg(undistorted_image, encoding='mono8')
        self.image_publisher.publish(image_msg)
    
    def joy_message_callback(self, data):
        # self.get_logger().warning(f"Fuck {data}")
        if data.buttons[0] == 1 and self.color_enabled == False:
        #     self.button_color_cam_counter += 1
        #     self.button_fisheye_cam_counter = 0
        #     print(self.button_color_cam_counter)
        # elif data.buttons[1] == 1:
        #     self.button_color_cam_counter = 0
        #     self.button_fisheye_cam_counter += 1
        #     print(self.button_fisheye_cam_counter)
        # if self.button_color_cam_counter == 20:
            self.set_t265_fisheye_camera_state(False)
            self.set_d455_color_camera_state(True)
            # self.button_color_cam_counter = 0
            self.color_enabled = True
            self.fisheye_enabled = False
        elif data.buttons[1] == 1 and self.fisheye_enabled == False:
        # elif self.button_fisheye_cam_counter == 20:
            self.set_t265_fisheye_camera_state(True)
            self.set_d455_color_camera_state(False)
            # self.button_fisheye_cam_counter = 0
            self.fisheye_enabled = True
            self.color_enabled = False
        elif data.buttons[2] == 1 and (self.fisheye_enabled == True or self.color_enabled == True):
            self.set_t265_fisheye_camera_state(False)
            self.set_d455_color_camera_state(False)
            self.fisheye_enabled = False
            self.color_enabled = False


def main(args=None):
    rclpy.init(args=args)

    video_stream_controller = VideoStreamController()

    rclpy.spin(video_stream_controller)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
