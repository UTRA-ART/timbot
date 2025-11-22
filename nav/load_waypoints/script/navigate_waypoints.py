from ament_index_python.packages import get_package_share_directory
from tf2_ros.transform_listener import TransformListener

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
        with open(base_dir + '/scripts/'+ self.static_waypoint_file) as f:
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



