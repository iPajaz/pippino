#! /usr/bin/env python3

"""
Description:
    Navigate to a charging dock once the battery gets low.
-------
Subscription Topics:
    Current battery state
    /battery_status - sensor_msgs/BatteryState
-------
Publishing Topics:
    Velocity command to navigate to the charging dock.
    /cmd_vel - geometry_msgs/Twist
-------
Author: Addison Sears-Collins
Website: AutomaticAddison.com
Date: November 16, 2021
"""

import time  # Time library

from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.executors import MultiThreadedExecutor
from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult # Helper module
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from geometry_msgs.msg import Twist # Velocity command
from sensor_msgs.msg import BatteryState # Battery status

# Holds the current state of the battery
this_battery_state = BatteryState()
prev_battery_state = BatteryState()

# Flag for detecting the change in the battery state
low_battery = True
low_battery_min_threshold = 0.25

class ConnectToChargingDockNavigator(Node):
    """
    Navigates and connects to the charging dock
    """      
    def __init__(self):

        # Initialize the class using the constructor
        super().__init__('connect_to_charging_dock_navigator')
    
        # Create a publisher
        # This node publishes the desired linear and angular velocity of the robot
        self.publisher_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)  
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.navigate_to_dock)
            
        # Declare velocities
        self.linear_velocity = 0.1
        self.angular_velocity = 0.0
        
    def navigate_to_dock(self):
        

    def connect_to_dock(self):  
        
        # While the battery is not charging
        # while this_battery_state.power_supply_status != 1:
    
        #   # Publish the current battery state
        #   self.get_logger().info('NOT CHARGING...')
        
        #   # Send the velocity command to the robot by publishing to the topic
        #   cmd_vel_msg = Twist()
        #   cmd_vel_msg.linear.x = self.linear_velocity
        #   cmd_vel_msg.angular.z = self.angular_velocity
        #   self.publisher_cmd_vel.publish(cmd_vel_msg)      
        #   time.sleep(0.1)
    
        # Stop the robot
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.publisher_cmd_vel.publish(cmd_vel_msg)
    
        self.get_logger().info('CHARGING...')
        self.get_logger().info('Successfully connected to the charging dock!')

            
def main(args=None):
    """
    Entry point for the program.
    """
    
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    try: 
    
        # Create the nodes
        connect_to_charging_dock_navigator = ConnectToChargingDockNavigator()
        
        # Set up mulithreading
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(connect_to_charging_dock_navigator)
        
        try:
            # Spin the nodes to execute the callbacks
            executor.spin_once()
        finally:
            # Shutdown the nodes
            executor.shutdown()
            connect_to_charging_dock_navigator.destroy_node()

    finally:
        # Shutdown
        rclpy.shutdown()

if __name__ == '__main__':
    main()
