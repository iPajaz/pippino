# Publishes a coordinate transformation between an ArUco marker and a camera
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
     
# Import the necessary ROS 2 libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image # Image is the message type

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

# Import Python libraries
import cv2 # OpenCV library
import numpy as np # Import Numpy library
import time
 
class FisheyeNode(Node):
    """
    Create an ArucoNode class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('fisheye_node')
 
        self.declare_parameter("camera_calibration_parameters_filename", "/home/michele/pippino_ws/src/pippino_navigation/config/calibration_T265_fisheye1.yaml")
        # self.declare_parameter("camera_calibration_parameters_filename", "/home/michele/pippino_ws/src/pippino_navigation/config/calibration_D455_640x360.yaml")
        # self.declare_parameter("image_topic", "/D455/color/image_raw")
        self.declare_parameter("image_topic", "/T265/fisheye1/image_raw")

        self.camera_calibration_parameters_filename = self.get_parameter(
            "camera_calibration_parameters_filename").get_parameter_value().string_value
        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value

        # Load the camera parameters from the saved file
        cv_file = cv2.FileStorage(
            self.camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ) 
        self.mtx = cv_file.getNode('K').mat()
        self.dst = cv_file.getNode('D').mat()

        self.mtx = np.asarray([[286.844207763672,0,427.016204833984],[0,286.76708984375,389.666412353516],[0,0,1]])
        self.dst = np.asarray([[-0.016013290733099],[0.057983260601759],[-0.0539293698966503],[0.0113047696650028]])

        # vvv = self.mtx[0,2]
        # self.mtx[0,2] = self.mtx[1,2]
        # self.mtx[1,2] = vvv

        print(self.mtx)
        print(self.dst)
        cv_file.release()
         
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, 
            image_topic, 
            self.listener_callback, 
            10)
        self.subscription # prevent unused variable warning
             
        # Used to convert between ROS and OpenCV images
        self.bridge = CvBridge()

        self.set_param_req1 = SetParameters.Request()
        self.set_param_req2 = SetParameters.Request()
        self.t265_param_cli = self.create_client(SetParameters, '/T265/T265/set_parameters')
        while not self.t265_param_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.set_t265_fisheye_camera_state(True)

    
    def set_t265_fisheye_camera_state(self, enabled):
            self.set_param_req1.parameters = [Parameter(name='enable_fisheye1', value=enabled).to_parameter_msg()]
            self.set_param_req2.parameters = [Parameter(name='enable_fisheye2', value=enabled).to_parameter_msg()]
            self.future = self.t265_param_cli.call_async(self.set_param_req1)
            self.future = self.t265_param_cli.call_async(self.set_param_req2)
            time.sleep(1)
    
    def listener_callback2(self, data):
        pass
        
    def listener_callback(self, data):
        """
        Callback function.
        """

        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        # breakpoint()


        # self.mtx = self.mtx*1.0
        scale_factor = 1
        # self.mtx[2,2] = 1.0
        # self.dst[0,0] *= 0.99
        time.sleep(0.1)

        # self.mtx[0, 0] *= 1.01
        # self.mtx[1, 1] *= 1.01
        # self.mtx[0, 2] *= 0.99
        # self.mtx[1, 2] *= 0.99

        # current_frame = cv2.flip(current_frame, 1)
        # self.mtx[1, 2] //= 2

        # tx_mat = np.eye(3)
        # tx_mat[0,2] = 2
        # tx_mat[1,2] = 2

        # self.mtx = np.matmul(self.mtx, tx_mat)
        # print(self.mtx)
        # final_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.mtx, self.dst, (int(scale_factor*848), int(scale_factor*800)), tx_mat, balance=1.0)

        # map_1, map_2 = cv2.fisheye.initUndistortRectifyMap(self.mtx, self.dst, np.eye(3), final_K, (scale_factor*848, scale_factor*800), cv2.CV_16SC2)
        # undistorted_image = cv2.remap(current_frame, map_1, map_2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # breakpoint()
        undistorted_image = cv2.fisheye.undistortImage(current_frame, self.mtx, self.dst, Knew=self.mtx, new_size=(1000, 1000))

        # Display image
        cv2.namedWindow("camera", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("camera", 848, 800)
        cv2.imshow("camera", undistorted_image)
        # cv2.namedWindow("camera2", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("camera2", 1240, 800)
        # cv2.imshow("camera2", current_frame)

        cv2.waitKey(1)
     
def main(args=None):
     
    # Initialize the rclpy library
    rclpy.init(args=args)
     
    # Create the node
    aruco_node = FisheyeNode()
     
    # Spin the node so the callback function is called.
    try:
        rclpy.spin(aruco_node)
    except KeyboardInterrupt:
        aruco_node.set_t265_fisheye_camera_state(False)
     
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    aruco_node.destroy_node()
     
    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()