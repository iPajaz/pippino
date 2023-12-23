import rclpy
from rclpy.node import Node

# MQTT
import autodock_action_client.mqtt as mqtt
from autodock_action_client.mqtt_settings import MQTT_BROKER, MQTT_AUTH

node_list = ['esp32_motor_controller', 'pippino_driver', 'video_stream_controller', 'aruco_detection_service', 'rosbridge_websocket', 'web_video_server', '_web_video_server', 'teleop_node', 'D455', 'T265', 'discoverer_server', 'navigation_client', 'cartographer_subscriber', 'joint_state_publisher', 'robot_state_publisher', 'autodock_action_server',  'controller_server', 'local_costmap', 'smoother_server', 'planner_server', 'global_costmap', 'behavior_server', 'discover_action_client', 'bt_navigator', 'bt_navigator_navigate_to_pose_rclcpp_node', 'bt_navigator_navigate_through_poses_rclcpp_node', 'waypoint_follower', 'velocity_smoother', 'autodock_action_client', 'lifecycle_manager_navigation', 'slam_toolbox', 'pippino_up_detection_node', 'ekf_filter_node', 'rplidar_node']


class MyNode(Node):
    def __init__(self):
        super().__init__('pippino_up_detection_node')
        self.get_logger().info('Pippino Up detection node initialized.')
        self.timer = self.create_timer(1, self.check_nodes)

    def check_nodes(self):
        node_names = [n[0] for n in self.get_node_names_and_namespaces()]
        if all(node in node_names for node in node_list):
            self.send_mqtt_message()
            rclpy.shutdown()

    def send_mqtt_message(self):
        mqtt.pub_single('homeassistant/pippino/status', client_id='pippino_status_server', payload=1, auth=MQTT_AUTH, hostname=MQTT_BROKER)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()