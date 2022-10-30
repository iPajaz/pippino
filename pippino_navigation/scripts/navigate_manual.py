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

rclpy.init()

# Declare velocities
linear_velocity = 0.0
angular_velocity = 0.15

def connect_to_dock(self):  
  
  a=0
  # While the battery is not charging
  while a<20:
    a+=1
    # Publish the current battery state
    Node.get_logger().info('NOT CHARGING...')
  
    # Send the velocity command to the robot by publishing to the topic
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = linear_velocity
    cmd_vel_msg.angular.z = angular_velocity
    publisher_cmd_vel.publish(cmd_vel_msg)      
    time.sleep(0.1)

  # Stop the robot
  cmd_vel_msg = Twist()
  cmd_vel_msg.linear.x = 0.0
  cmd_vel_msg.angular.z = 0.0
  publisher_cmd_vel.publish(cmd_vel_msg)

  Node.get_logger().info('CHARGING...')
  Node.get_logger().info('Successfully connected to the charging dock!')
# try:
  
    # Create the nodes
# Create a publisher
# This node publishes the desired linear and angular velocity of the robot
publisher_cmd_vel = Node.create_publisher(Node,msg_type=Twist, topic='/cmd_vel', qos_profile=10)  
timer_period = 0.1
timer = create_timer(timer_period, navigate_to_dock)
  



Node.get_logger().info('Navigating to the charging dock...')

# Launch the ROS 2 Navigation Stack
navigator = BasicNavigator()

# Wait for navigation to fully activate. Use this line if autostart is set to true.
navigator.waitUntilNav2Active()

# If desired, you can change or load the map as well
# navigator.changeMap('/path/to/map.yaml')

# You may use the navigator to clear or obtain costmaps
# navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
# global_costmap = navigator.getGlobalCostmap()
# local_costmap = navigator.getLocalCostmap()

# Set the robot's goal pose
goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.header.stamp = navigator.get_clock().now().to_msg()
goal_pose.pose.position.x = 0.0
goal_pose.pose.position.y = 0.5
goal_pose.pose.position.z = 0.0
goal_pose.pose.orientation.x = 0.0
goal_pose.pose.orientation.y = 0.0
goal_pose.pose.orientation.z = 0.0
goal_pose.pose.orientation.w = 1.0

# Go to the goal pose
navigator.goToPose(goal_pose)

i = 0

# Keep doing stuff as long as the robot is moving towards the goal
while not navigator.isNavComplete():
  # Do something with the feedback
  i = i + 1
  feedback = navigator.getFeedback()
  if feedback and i % 5 == 0:
    print('Distance remaining: ' + '{:.2f}'.format(
      feedback.distance_remaining) + ' meters.')

    # Some navigation timeout to demo cancellation
    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
      navigator.cancelNav()

# Do something depending on the return code
result = navigator.getResult()
if result == NavigationResult.SUCCEEDED:
  print('Successfully reached charging dock staging area...')
  low_battery = False
  connect_to_dock()
elif result == NavigationResult.CANCELED:
  print('Goal was canceled!')
elif result == NavigationResult.FAILED:
  print('Goal failed!')
else:
  print('Goal has an invalid return status!')  
    




    
# finally:
#   # Shutdown
rclpy.shutdown()

