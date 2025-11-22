
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

        self.tf = TransformListener()
        self.publisher = node.create_publisher(Bool, '/waypoint_int', 10)

        self.ramp_naving = False
        self.cv_ramp_naving = th.Condition()

        self.ramp_wp_sub = node.create_subscription(Bool, 'ramp_naving', self.ramp_naving_callback)


