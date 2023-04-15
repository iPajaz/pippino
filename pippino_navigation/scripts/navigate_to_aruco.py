import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Twist # Velocity command

from transforms3d.euler import quat2euler


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
        self.timer = self.create_timer(0.1, self.on_timer)
        self.delta_x = 0
        self.delta_y = 0
        self.angle_to_charging_position = 0

        self.linear_velocity = 0.05  # meters per second
        self.angular_velocity = 0.05 # radians per second

        # Declare distance metrics in meters
        self.distance_goal_tolerance1 = 0.15
        self.distance_goal_tolerance2 = 0.03
        self.reached_distance_goal = False     

        # Declare angle metrics in radians
        self.heading_tolerance = 0.04
        self.yaw_goal_tolerance = 0.05

        self.charging_position_rough_reached = False
        self.charging_position_reached = False

        self.charging_aruco_visible = self.create_subscription(
            Bool,
            'charging_station_aruco_visible',
            self.charging_station_aruco_visible_callback,
            10)

    def charging_station_aruco_visible_callback(self, msg):
        self.charging_aruco_visible =  msg.data

    def on_timer(self):
        if not self.charging_aruco_visible:
            self.stop_robot()
        elif not self.charging_position_rough_reached:
            self.go_to_location('charging_position_rough', distance_tolerance=0.15, heading_tolerance=0.05, final_yaw_tolerance=0.0, final_yaw_correction=False)
        elif not self.charging_position_reached:
            self.go_to_location('charging_position', distance_tolerance=0.03, heading_tolerance=0.05, final_yaw_tolerance=0.01, final_yaw_correction=True)
        else:
            print("Charging position reached!")

    def stop_robot(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        
        # Publish the velocity message  
        self.publisher_cmd_vel.publish(cmd_vel_msg) 

    def get_transform(self, target_frame):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = target_frame
        to_frame_rel = 'base_link'

        # Charging position means the goal for the robot is to have base_link overlap with charging_position in x and y

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            self.stop_robot() 
            return

        # print(t.header.stamp, rclpy.time.Time())

        self.delta_y = t.transform.translation.y  # - 0.0027
        #y vertical, z distance
        # delta_y = t.transform.translation.y
        self.delta_x = t.transform.translation.x  # - 0.07 - 0.14
        # print(t.transform.rotation)
        _,_,self.yaw_to_position = quat2euler([t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z], axes='syxz')
        # a,b,c = quat2euler([t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z], axes='syxz')
        # r,p,y = (val*180/3.141592 for val in [r,p,y])
        # print(f"x = {self.delta_x:.03f}, y = {self.delta_y:.03f}, z={t.transform.translation.z:.03f} theta = {self.yaw_to_position:.03f}")
        # print(f"a = {a:.03f}, b = {b:.03f}, c={c:.03f}")


        # val = input("Go? ")
        # if val == 'y':
        #     pass
        # self.get_logger().info(
        #         f'x={t.transform.translation.x:f} y={t.transform.translation.y:f} z={t.transform.translation.z:f}')

        # delta_x is distance in dir along pippino length, y along pippino width from charging position
        # angle is angle between pippino and aruco

        return self.delta_x, self.delta_y, self.yaw_to_position


    def get_distance_to_goal(self, delta_x, delta_y):
        """Get the distance between pippino and the charging position.
        """
        return math.sqrt(delta_y**2+delta_x**2)

    def get_heading_error(self, delta_x, delta_y):
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

    # def get_radians_to_goal(self):
    #     """Get the yaw goal angle error in radians
    #     """
    #     return self.angle_to_charging_position

    def go_to_location(self, target_location, distance_tolerance, heading_tolerance, final_yaw_tolerance, final_yaw_correction):
        """
        Go to the line that is perpendicular to the AR tag
        """
        delta_x_tot = delta_y_tot = yaw_to_location_tot = 0
        for _ in range(5):
            try:
                delta_x,delta_y,yaw_to_location = self.get_transform(target_location)
            except TypeError:
                return
            delta_x_tot += delta_x
            delta_y_tot += delta_y
            yaw_to_location_tot += yaw_to_location

        delta_x = delta_x_tot/5
        delta_y = delta_y_tot/5
        yaw_to_location = yaw_to_location_tot/5

        cmd_vel_msg = Twist()
         
        # angular.z > 0 => goes left

        distance_to_goal = self.get_distance_to_goal(delta_x, delta_y)
        heading_error = self.get_heading_error(delta_x, delta_y)
        yaw_goal_error = yaw_to_location

        print(f"d={math.fabs(distance_to_goal)*100:.02f},{distance_tolerance}\t, herr={heading_error*100:.02f}\t,y={yaw_goal_error*100:.02f}\t,dx={self.delta_x*100:.02f}\t,dy={self.delta_y*100:.02f}\t")
        # If we are not yet at the position goal
        if (math.fabs(distance_to_goal) > distance_tolerance):
            print("step 1")
            # If the robot's heading is off, fix it
            if (math.fabs(heading_error) > heading_tolerance):
             
                if heading_error > 0:
                    # Go to the right
                    cmd_vel_msg.angular.z = self.angular_velocity
                else:
                    cmd_vel_msg.angular.z = -self.angular_velocity
            else:
                cmd_vel_msg.linear.x = max(min(self.linear_velocity*math.fabs(distance_to_goal)*2, 0.2), 0.015)
                # print(f"v={self.linear_velocity*math.fabs(distance_to_goal)*2}")

        # Orient towards the yaw goal angle
        elif ((math.fabs(yaw_goal_error) > final_yaw_tolerance) and final_yaw_correction is True):
             
            print("step 2")
            if yaw_goal_error > 0:
                cmd_vel_msg.angular.z = self.angular_velocity
            else:
                cmd_vel_msg.angular.z = -self.angular_velocity
             
            # self.reached_distance_goal = True
         
        # Goal achieved, go to the next goal
        elif final_yaw_correction is True:
            self.charging_position_reached = True
        else:
            print("step 3")
            # Go to the next goal
            # self.goal_idx = self.goal_idx + 1   
            # self.get_logger().info('Arrived at perpendicular line.')
            # self.reached_distance_goal = False    
            # Stop the robot
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.angular.z = 0.0
            self.charging_position_rough_reached = True
            
        # Publish the velocity message  
        self.publisher_cmd_vel.publish(cmd_vel_msg) 

        # self.get_logger().info('Robot is idle...')
             

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