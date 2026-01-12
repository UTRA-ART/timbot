#!/usr/bin/env python3

#############################################################################
# Script handles three different navigation requests via a single ROS service
# named 'rover_navigation'. User can specify the type of goal: 'abs', 'rel', 'gps'
# and provide the corresponding coordinates. 'handle_navigation_request' method
# processes these different requests and navigates the rover.


######## Request Format ########
# Only formats below works properly. Format such as:
# rosservice call /rover_navigation "goal_type: 'abs' goal: x: 5.0, y: 0.0"

# If a single line does not work, uncomment blocks (e.g. lines 18-21) to test in terminals
# TO SET A NEW GOAL IN THE MIDDLE, open new terminal and 1. source devel/setup.bash 2. input request format 3. <enter>

# 1. Absolute goals in map frame ('abs')
# rosservice call /rover_navigation "goal_type: 'abs'
# goal:
#   x: 5.0
#   y: 0.0"

# 2. Relative goals in map frame ('rel')
# rosservice call /rover_navigation "goal_type: 'rel'
# goal:
#   x: 10.0
#   y: 0.0"

# 3. Single gps goal ('gps')
# rosservice call /rover_navigation "goal_type: 'gps'
# goal:
#   x: -79.3904467252
#   y: 43.6570767441"

# Example:
# rosservice call /rover_navigation "goal_type: ''     
# goal:
#  x: 0.0
#  y: 0.0
#  z: 0.0" 



import rclpy
from rclpy.node import Node

from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

import tf2_ros
from tf2_ros import TransformListener

import utm
from load_waypoints.srv import RoverNavigation, RoverNavigationResponse

class RoverNavigator(Node):
    def __init__(self):
        super().__init__('nav_control')
        tf_buffer = tf2_ros.Buffer()
        self.tf = TransformListener(tf_buffer, self)
        self.current_pos = None
        self.active = False  # Indicates if the rover is actively navigating to a goal

        # Create service for handling navigation requests
        self.navigation_service = self.create_service(RoverNavigation, 'rover_navigation', self.handle_navigation_request)

        self.subscription = self.create_subscription(PoseStamped, '/tracked_pose', self.odometry_callback, 10)
        
    def odometry_callback(self, msg):
        self.current_pos = (msg.pose.position.x, msg.pose.position.y)
        
    def calculate_relative_coords(self, target_pos):
        if self.current_pos is None:
            self.get_logger().error("Current position not yet initialized.")
            return None
        
        relative_coords = [target_pos[0] - self.current_pos[0], target_pos[1] - self.current_pos[1]]
        return relative_coords

    def send_goal_to_move_base(self, goal_pos):
        action_client = ActionClient(self, NavigateToPose, '/move_base')
        action_client.wait_for_server()

        goal = NavigateToPose()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal.target_pose.pose.position.x = goal_pos[0]
        goal.target_pose.pose.position.y = goal_pos[1]
        goal.target_pose.pose.orientation.w = 1.0  

        future = action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        return action_client

    def navigate_to_goal(self, goal_pos):
        self.get_logger().info(f"Navigating to goal: (x: {goal_pos[0]}, y: {goal_pos[1]})")
        self.send_goal_to_move_base(goal_pos)

    def get_pose_from_gps(self, longitude, latitude):
        '''Converts GPS coordinates to map frame, same method from 'navigate_waypoints.py'''
        utm_coords = utm.from_latlon(latitude, longitude)  # Lat and Lon transformed into /utm
        utm_pose = PoseStamped() # create PoseStamped message with utm coords
        utm_pose.header.frame_id = 'utm'
        utm_pose.pose.position.x = utm_coords[0]
        utm_pose.pose.position.y = utm_coords[1]
        utm_pose.pose.orientation.w = 1.0  # To make sure it's right side up

        p_in_map = self.tf.transformPose("/map", utm_pose)   # self.tf (listener) listens for transforms between frames
                                                            # transform utm pose to map frame
        return p_in_map.pose.position.x, p_in_map.pose.position.y

    def handle_navigation_request(self, req):
        if req.goal_type == 'abs':
            goal_pos = (req.goal.x, req.goal.y) # if 'goal_type' = 'abs', directly use coordinates as target
                                                # rover will move to target in /map frame
        elif req.goal_type == 'rel':
            if self.current_pos is None:
                return RoverNavigationResponse(False, "Current position not yet initialized.")
            goal_pos = (self.current_pos[0] + req.goal.x, self.current_pos[1] + req.goal.y) # if 'goal_type' = 'rel', calculate target
                                                                                            # relative to rover's current position
        elif req.goal_type == 'gps':
            goal_pos = self.get_pose_from_gps(req.goal.x, req.goal.y)
        else:
            return RoverNavigationResponse(False, "Invalid goal type.")
        
        self.navigate_to_goal(goal_pos)
        return RoverNavigationResponse(True, "Navigating to goal.")
    
    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        # Start timer for timeout (600 seconds)
        self.timeout_timer = self.create_timer(600.0, self.timeout_cb)

        # Start waiting for result
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_cb)

    def timeout_callback(self):
        self.get_logger().info("Time out!")

        # Stops timer and goal
        self.timeout_timer.cancel()
        self.goal_handle.cancel_goal_async()

    def result_callback(self, future):
        # Stop timeout timer since result is received
        self.timeout_timer.cancel()
        self.get_logger().info("Reached nav goal!")

##############################################################################################

if __name__ == "__main__":#
	# Initializing nav_control node
	rclpy.init()

	navigator = RoverNavigator()

	# Keep node running until shutting down
	rclpy.spin(navigator)