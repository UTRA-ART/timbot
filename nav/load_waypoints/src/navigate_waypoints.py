from ament_index_python.packages import get_package_share_directory
from tf2_ros.transform_listener import TransformListener
from rclpy.action import ActionClient
import time

import threading as th


class NavigateWaypoints:
    def __init__(self, static_waypoint_file, max_time_for_transform):
        self.waypoints = dict()
        self.static_waypoint_file = static_waypoint_file
        self.max_time_for_transform = max_time_for_transform
        self.waited_for_transform = False

        # declare node
        self.launch_state = node.declare_parameter('/load_waypoints_server/launch_state', """default value""")
        self.ignore_lidar = False
        self.start_direction = 1
        self.laps = 0

        self.populate_waypoint_dict()

        self.current_lap = 0
        self.curr_waypoint_idx = 0 if self.start_direction == 1 else len(self.waypoints) - 2
        node.get_logger().info("First goal: %s" % (self.curr_waypoint_idx))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = node.create_publisher(Bool, '/waypoint_int', 10)

        self.ramp_naving = False
        self.cv_ramp_naving = th.Condition()

        self.ramp_wp_sub = node.create_subscription(Bool, 'ramp_naving', self.ramp_naving_callback)

        # Used to wait for result after async_send_goal
        self.result_received = 0

    def populate_waypoint_dict(self):
        '''
        Description: 
            Used to populate the waypoint dictionary with i) the static waypoints obtained during competition time and 
            ii) the first gps coordinate that acts as the final waypoint. 
        '''

        # Load waypoints into waypoint_data, using ament_index_python as alternative to rospkg
        # https://robotics.stackexchange.com/questions/86532/ros2-equivalent-of-rospackagegetpath
        # May not be exactly interchangeable, fix later if necessary

        base_dir = get_package_share_directory('load_waypoints')

        # Load in static waypoints (provided at competition time) 
        with open(base_dir + '/jsons/'+ self.static_waypoint_file) as f:
            try:
                waypoint_data = json.load(f)
            except:
                node.get_logger().info("Invalid JSON")
                sys.exit(1)

        self.start_direction = 1 if waypoint_data["start_direction"] == "north" else -1
        self.laps = waypoint_data["laps"]

        node.get_logger().info("start_direction: %s" % (self.start_direction))

        # Call method to wait for transform 
        self.waited_for_transform = self.wait_for_utm_transform()

        # Check if successfully waited for the transform within the time limit. If successful, continue populating the waypoint dict. 
        if self.waited_for_transform:
            # After waiting UTM transform, capture a message from the gps/fix topic
            # REPLACE QOS_PROFILE
            gps_info = rclpy.wait_for_message(NavSatFix, 'navigate_waypoints', 'gps/fix', """qos_profile""", 5)
        else:
            node.get_logger().info("Waiting for transform from /map to /utm timed out!")
        
        # Add additional waypoints to the corners of the course to avoid incorrect shortcuts
        if waypoint_data["add_corners"]:
            self.add_corners(waypoint_data, gps_info)
        else:
            # Parse through json data and create list of lists holding all waypoints
            for waypoint in waypoint_data["waypoints"]:
                self.waypoints[waypoint['id']] = waypoint
        
        # Append the starting gps coordinate to the waypoints dict as the final waypoint
        last_coord_idx = len(self.waypoints) 

        # Append a final waypoint to return to the start (i.e. waypoint to return to start)
        self.waypoints[last_coord_idx] = {
            'id': last_coord_idx, 
            'longitude': gps_info.longitude, 
            'latitude': gps_info.latitude, 
            'description': 'Initial start location', 
            'frame_id': waypoint_data["waypoints"][0]["frame_id"] # For now is 'map'
        }

        # Show waypoints
        node.get_logger().info("Successfully loaded waypoints dict")

        return
    
    def add_corners(self, waypoint_data, gps_info):
        '''
        Description: 
            Add corner waypoints in the lanes to better navigate rover. 
        '''
        is_sim = self.launch_state == "sim"
        frame = waypoint_data["waypoints"][0]["frame_id"]
        j = 0

        # Account for whether the state is sim because map is rotated to face East instead of North
        for i in range(len(waypoint_data["waypoints"]) + 3):
            if i == 0:
                self.waypoints[i] = {
                    'id': i, 
                    'longitude': -79.3905355 if is_sim else gps_info.longitude, 
                    'latitude': gps_info.latitude + 0.00001 if is_sim else waypoint_data["waypoints"][0]["latitude"], 
                    'description': "First Corner", 
                    'frame_id': frame
                }
            elif i == 5:
                self.waypoints[i] = {
                    'id': i, 
                    'longitude': -79.38998072 if is_sim else waypoint_data["waypoints"][3]["longitude"], 
                    'latitude': 43.65714925 if is_sim else waypoint_data["waypoints"][3]["latitude"] - 0.000036, 
                    'description': "Third Corner", 
                    'frame_id': frame
                }
            elif i == 6:
                self.waypoints[i] = {
                    'id': i, 
                    'longitude': waypoint_data["waypoints"][3]["longitude"] if is_sim else gps_info.longitude, 
                    'latitude':  gps_info.latitude - 0.00001 if is_sim else waypoint_data["waypoints"][3]["latitude"], 
                    'description': "Fourth Corner", 
                    'frame_id': frame
                }
            else:
                self.waypoints[i] = waypoint_data["waypoints"][j]
                self.waypoints[i]["id"] = i
                j += 1

    def wait_for_utm_transform(self):
        '''
        Description: 
            Used to wait for a transform from the /map frame to /utm frame (which indicates that the GPS is ready). This accounts/simulates for gps start-up time. 
            Once the transform is detected, this function will exit. 
        '''

        # Initialize transform listener
        buffer = Buffer()
        listener = TransformListener(buffer, self)

        rate = node.create_rate(10.0)

        start_time = self.get_clock().now()

        try:
            while rclpy.ok():
                time_waited = self.get_clock().now() - start_time
                if (time_waited) >= self.max_time_for_transform:
                    node.get_logger().info("Waiting for transform timed out. Time waited for transform: %s s"%(time_waited))
                    waited_for_transform = False
                    break
                else:
                    try:
                        now = self.get_clock().now()

                        # Wait for transform from /map to /utm
                        buffer.lookup_transform("/map", "/utm", now, 5.0)
                        node.get_logger().info("Transform found. Time waited for transform: %s s"%(self.get_clock().now() - start_time))
                        waited_for_transform = True
                        break
                    except:
                        pass
                
                rate.sleep()
        except:
            pass
        
        return waited_for_transform
    
    def get_next_waypoint(self):
        waypoint = self.waypoints[self.curr_waypoint_idx]
        node.get_logger().info("Next Goal: %s"%(waypoint["description"]))
        if self.curr_waypoint_idx == 3 and self.start_direction == 1: # curr_waypoint_idx = 2 means heading towards id 2
            self.ignore_lidar = True 
        elif self.curr_waypoint_idx == 2 and self.start_direction == -1:
            self.ignore_lidar = True 
        else:
            self.ignore_lidar = False

        for i in range(10):
            publisher.publish(self.ignore_lidar)

        self.curr_waypoint_idx += self.start_direction
        if self.curr_waypoint_idx < 0 and self.current_lap < self.laps:
            self.current_lap += 1
            self.curr_waypoint_idx = len(self.waypoints) - 1
        elif self.curr_waypoint_idx >= len(self.waypoints) and self.current_lap < self.laps:
            self.current_lap += 1
            self.curr_waypoint_idx = 0
        
        return waypoint
    
    def get_pose_from_gps(self, longitude, latitude, frame, pose_test_var = None):
        '''converts gps coordinates to frame (odom,map,etc)'''
        
        # create PoseStamped message to set up for do_transform_pose
        utm_coords = utm.from_latlon(latitude, longitude)#latitude and longitude transformed into UTM
        utm_pose = PoseStamped()
        utm_pose.header.frame_id = 'utm'
        utm_pose.pose.position.x = utm_coords[0]
        utm_pose.pose.position.y = utm_coords[1]
        utm_pose.pose.orientation.w = 1.0 # to make sure its right side up

        p_in_frame = self.buffer.transform(utm_pose ,"/"+frame, 1.0)

        return p_in_frame

    def send_and_wait_goal_to_move_base(self, curr_waypoint):
        # Create an action client called "move_base" with action definition file "MoveBaseAction"
        action_client = ActionClient(self, MoveBaseAction, '/move_base')

        # Waits until the action server has started up and started listening for goals.
        action_client.wait_for_server()

        # Creates a new goal with the NavigateToPose constructor
        goal = NavigateToPose()
        goal.pose.header.frame_id = curr_waypoint["frame_id"]
        goal.pose.header.stamp = self.get_clock().now()

        #while not reached Goal, resend the goal. 
        #if finished goal, send the next goal and start again. 
        finished_within_time = 0

        times = 0

        # Send goals repeatedly  
        while 1:
            # Set goal position and orientation
            pose = self.get_pose_from_gps(curr_waypoint["longitude"], curr_waypoint["latitude"], curr_waypoint["frame_id"])
            goal.pose = pose.pose

            # Sends goal and waits until the action is completed (or aborted if it is impossible)
            goal_handle = action_client.send_goal_async(goal)
            action_client.async_get_result(goal_handle, self.recieve_result)

            elapsed_time = 0
            while (time.sleep(0.01)):
                if result_received == 1:
                    result_received = 0
                    break
                elapsed_time += 0.01

                if elapsed_time >= 5:
                    break
            
            with self.cv_ramp_naving:
                if self.ramp_naving:
                    node.get_logger().info("Normal nav INTERRUPTED") # ramp_navigate.cpp takes over
                    self.cv_ramp_naving.wait_for(lambda : not self.ramp_naving) # Stalls here (thread blocked) until ramp nav completed 
                    node.get_logger().info("Returning to waypoint navigation")
                    break
                elif finished_within_time:
                    node.get_logger().info("Reached nav goal")
                    break
                else:
                    times += 1
    
    def recieve_result():
        self.result_received = 1
        return 1

    def navigate_waypoints(self):
        while True:
            curr_waypoint = self.get_next_waypoint()
            self.send_and_wait_goal_to_move_base(curr_waypoint)
            if self.ramp_naving:
                break

            if (self.current_lap >= self.laps):
                break
    
    # Constanting updating the threading conditions
    def ramp_naving_callback(self, ramp_naving):
        with self.cv_ramp_naving:
            self.ramp_naving = ramp_naving.data
            if not self.ramp_naving:
                self.cv_ramp_naving.notify_all() # Notifies blocked threads to recheck their condition
    
    if __name__ == "__main__":
        # Pick json file with desired GPS coordinates
        launch_state = node.declare_parameter('/load_waypoints_server/launch_state', """default value""")
        launch_state = "IGVC"
        if launch_state == "sim":
            static_waypoint_file = 'static_waypoints_pavement.json'
        else:
            static_waypoint_file = 'IGVC_practice.json'

        rclpy.create_node('navigate_waypoints')
        waypoints = NavigateWaypoints(static_waypoint_file, max_time_for_transform=60.0)
    
        # waypoints.navigate_waypoints()
        t = th.Thread(target=waypoints.navigate_waypoints)
        t.start()
        rclpy.spin(waypoints)
        t.join()
        rospy.create_node('Finished Navigating!!')

            



