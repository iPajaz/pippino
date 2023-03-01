import math
from enum import Enum
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist # Velocity command

from transforms3d.euler import quat2euler
from themstates import StateMachine

class NavToDockStates(Enum):
    IDLE = 0
    ARUCO_FOUND = 1
    ARUCO_VISIBLE = 2
    ROUGH_POSITION_REACHED = 3
    CHARGING_POSITION_REACHED = 4
    CHARGING_YAW_FIXED = 5
    DONE = 6

class ArucoFrameListener(Node):

    def __init__(self):
        super().__init__('charuco_frame_listener')

        # Create a publisher
        # This node publishes the desired linear and angular velocity of the robot
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(0.2, self.on_timer)
        # self.delta_x = 0
        # self.delta_y = 0
        self.angle_to_charging_position = 0

        self.linear_velocity = 0.08  # meters per second
        self.angular_velocity = 0.05 # radians per second

        # Declare distance metrics in meters
        self.rough_position_distance_tolerance = 0.15
        self.rough_position_heading_tolerance = 0.05
        self.final_position_distance_tolerance = 0.02
        self.final_position_heading_tolerance = 0.02
        self.final_position_yaw_tolerance = 0.01
        self.reached_distance_goal = False     

        # Declare angle metrics in radians
        self.charging_aruco_visible_subscriber = self.create_subscription(
            Bool,
            'charging_station_aruco_visible',
            self.charging_station_aruco_visible_callback,
            10)

        self.cmd_vel_msg = Twist()
        self.state = NavToDockStates.IDLE
        self.charging_aruco_visible = False

    def reset_state(self):
        # States
        self.charging_position_rough_reached = False
        self.charging_position_reached = False
        self.charging_aruco_found = False

    def charging_station_aruco_visible_callback(self, msg):
        self.charging_aruco_visible =  msg.data

    def on_timer(self):
        print(self.state)
        # self.state = self.next_state
        
        if self.state == NavToDockStates.IDLE:
            if self.charging_aruco_visible:
                self.state = NavToDockStates.ARUCO_FOUND
        elif self.state == NavToDockStates.ARUCO_FOUND:
            if self.move_towards_location('charging_position_rough', distance_tolerance=self.rough_position_distance_tolerance, heading_tolerance=self.rough_position_heading_tolerance):
                self.state = NavToDockStates.ROUGH_POSITION_REACHED
        elif self.state == NavToDockStates.ROUGH_POSITION_REACHED:
            if self.move_towards_location('charging_position', distance_tolerance=self.final_position_distance_tolerance, heading_tolerance=self.final_position_heading_tolerance):
                self.state = NavToDockStates.CHARGING_POSITION_REACHED
        elif self.state == NavToDockStates.CHARGING_POSITION_REACHED:
            if self.correct_yaw_to_dock('charging_position', final_yaw_tolerance=self.final_position_yaw_tolerance):
                self.state = NavToDockStates.CHARGING_YAW_FIXED
        elif self.state == NavToDockStates.CHARGING_YAW_FIXED:
            distance_to_location, _, _ = self.get_delta_to_location('charging_position')
            if math.fabs(distance_to_location) > self.final_position_distance_tolerance:
                self.state = NavToDockStates.ROUGH_POSITION_REACHED
            else:
                self.state = NavToDockStates.DONE
        elif self.state == NavToDockStates.DONE:
            self.stop_robot()

    def get_transform(self, target_frame):
        from_frame_rel = target_frame
        to_frame_rel = 'base_link'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())  # , rclpy.duration.Duration(seconds=1))
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            self.stop_robot()
            raise TransformException
            return

        # print(t.header.stamp, rclpy.time.Time())

        delta_y = t.transform.translation.y# - 0.0027
        delta_x = t.transform.translation.x# - 0.07 - 0.14
        _,_,yaw_to_position = quat2euler([t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z], axes='syxz')

        return delta_x, delta_y, yaw_to_position


    # def move_to_location(self, target_location, distance_tolerance, heading_tolerance, final_yaw_tolerance, final_yaw_correction):
    #     location_reached = False
    #     while not location_reached:
    #         location_reached = move_towards_location(target_location, distance_tolerance, heading_tolerance)
    #     if final_yaw_correction:
    #         while not yaw_reached:
    #             yaw_reached = correct_yaw_to_dock(final_yaw_tolerance)

    def move_towards_location(self, target_location, distance_tolerance, heading_tolerance):
        """
        Go to the target location
        """
        distance_to_location, heading_delta_to_location, yaw_delta_to_location = self.get_delta_to_location(target_location)

        if (math.fabs(distance_to_location) > distance_tolerance):
            # If we are not yet at the position goal
            if (math.fabs(heading_delta_to_location) > heading_tolerance):
                print(f"heading delta = {heading_delta_to_location}")
                # Orient robot towards goal location
                self.correct_yaw(heading_delta_to_location)
            else:
                # Move towards goal location
                self.correct_distance(distance_to_location)
                # print(f"v={self.linear_velocity*math.fabs(distance_to_location)*2}")
            return False

        return True

    def correct_yaw_to_dock(self, target_location, final_yaw_tolerance):
        _, _, yaw_delta_to_location = self.get_delta_to_location(target_location)
        # Orient robot towards final yaw, facing charging dock
        if ((math.fabs(yaw_delta_to_location) > final_yaw_tolerance)):
            # self.reached_distance_goal = True
            self.correct_yaw(yaw_delta_to_location)
            return False
        return True

    def stop_robot(self):
        self.cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_msg.angular.z = 0.0
        self.publisher_cmd_vel.publish(self.cmd_vel_msg) 

    def correct_yaw(self, yaw_delta):
        print("Correcting Rotation")
        self.cmd_vel_msg.linear.x = 0.0
        if yaw_delta > 0:
            self.cmd_vel_msg.angular.z = self.angular_velocity
        else:
            self.cmd_vel_msg.angular.z = -self.angular_velocity
        self.publisher_cmd_vel.publish(self.cmd_vel_msg)

    def correct_distance(self, distance_to_location):
        print("Correcting Distance")
        self.cmd_vel_msg.linear.x = max(min(self.linear_velocity*math.fabs(distance_to_location)*2, 0.2), 0.015)
        self.cmd_vel_msg.angular.z = 0.0
        self.publisher_cmd_vel.publish(self.cmd_vel_msg)

    def get_distance(self, delta_x, delta_y):
        """Get the distance from delta_x and delta_y.
        """
        return math.sqrt(delta_y**2+delta_x**2)

    def get_heading_delta(self, delta_x, delta_y):
        """
        Get the heading error in radians
        """
        desired_heading = math.atan2(delta_y, delta_x) 
        heading_error = desired_heading
         
        # Make sure the heading error falls within -PI to PI range
        if (heading_error > math.pi):
            heading_error = heading_error - (2 * math.pi)
        if (heading_error < -math.pi):
            heading_error = heading_error + (2 * math.pi)
         
        return heading_error

    def get_delta_to_location(self, target_location):
        delta_x_sum = delta_y_sum = delta_yaw_sum = 0
        # Average of 5 measurements
        for _ in range(5):
            try:
                delta_x,delta_y,delta_yaw = self.get_transform(target_location)
            except TypeError:
                raise TypeError
                return
            delta_x_sum += delta_x
            delta_y_sum += delta_y
            delta_yaw_sum += delta_yaw
        delta_x = delta_x_sum/5
        delta_y = delta_y_sum/5
        delta_yaw = delta_yaw_sum/5

        # angular.z > 0 => goes left

        distance_to_location = self.get_distance(delta_x, delta_y)
        # Delta direction to the xy position
        heading_delta_to_location = self.get_heading_delta(delta_x, delta_y)
        # Delta direction wrt target location direction (the axes position of the target_location)
        yaw_delta_to_location = delta_yaw

        print(f"d={math.fabs(distance_to_location)*100:.02f}, herr={heading_delta_to_location*100:.02f}\t,y={yaw_delta_to_location*100:.02f}\t,dx={delta_x*100:.02f}\t,dy={delta_y*100:.02f}\t")
        return distance_to_location, heading_delta_to_location, yaw_delta_to_location

def main():
    rclpy.init()
    node = ArucoFrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
  main()