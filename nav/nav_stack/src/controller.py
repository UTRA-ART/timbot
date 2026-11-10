import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
# ADD NECCESSARY IMPORTS HERE

class NavController(Node):
    def __init__(self):
        super().__init__('nav_controller')

        # Gets all the parameters from config specified in launch file
        self.declare_parameter('configuration.map_update_frequency', 5.0)
        self.declare_parameter('configuration.controller_update_frequency', 20.0)
        self.declare_parameter('configuration.robot_position_topic', '/odometry/global')
        self.declare_parameter('configuration.obstacle_map_topic', '/map')
        self.declare_parameter('configuration.path_map_topic', '/nav/planned_path')
        self.declare_parameter(
            'configuration.velocity_output_topic', '/nav_vel'
        )

        self.declare_parameter('mapping.inflation_radius', 1.0)
        self.declare_parameter('mapping.inflation_linear', False)
        self.declare_parameter('mapping.inflation_max', 100)
        self.declare_parameter('mapping.inflation_min', 20)
        self.declare_parameter('mapping.robot_radius', 1.0)
        self.declare_parameter('mapping.map_resolution', 1.0)

        self.declare_parameter('constraints.goal_tolerance', 1.0)
        self.declare_parameter(
            'constraints.max_sideways_linear_acceleration', 1.0
        )
        self.declare_parameter(
            'constraints.max_forwards_linear_acceleration', 1.0
        )
        self.declare_parameter('constraints.max_angular_acceleration', 1.0)
        self.declare_parameter('constraints.max_linear_velocity', 1.0)
        self.declare_parameter('constraints.max_angular_velocity', 1.0)

        self.declare_parameter('controller.lookahead_distance', 1.0)
        self.declare_parameter('controller.kP', 1.0)
        self.declare_parameter('controller.kI', 1.0)
        self.declare_parameter('controller.kD', 1.0)

        self.map_update_frequency = float(
            self.get_parameter('configuration.map_update_frequency').value
        )
        self.controller_update_frequency = float(
            self.get_parameter('configuration.controller_update_frequency').value
        )
        self.robot_position_topic = self.get_parameter(
            'configuration.robot_position_topic'
        ).value
        self.obstacle_map_topic = self.get_parameter(
            'configuration.obstacle_map_topic'
        ).value
        self.path_map_topic = self.get_parameter(
            'configuration.path_map_topic'
        ).value
        self.velocity_output_topic = self.get_parameter(
            'configuration.velocity_output_topic'
        ).value

        self.inflation_radius = float(
            self.get_parameter('mapping.inflation_radius').value
        )
        self.inflation_linear = bool(
            self.get_parameter('mapping.inflation_linear').value
        )
        self.inflation_max = int(
            self.get_parameter('mapping.inflation_max').value
        )
        self.inflation_min = int(
            self.get_parameter('mapping.inflation_min').value
        )
        self.robot_radius = float(self.get_parameter('mapping.robot_radius').value)
        self.map_resolution = float(
            self.get_parameter('mapping.map_resolution').value
        )

        self.goal_tolerance = float(
            self.get_parameter('constraints.goal_tolerance').value
        )
        self.max_sideways_linear_acceleration = float(
            self.get_parameter(
                'constraints.max_sideways_linear_acceleration'
            ).value
        )
        self.max_forwards_linear_acceleration = float(
            self.get_parameter(
                'constraints.max_forwards_linear_acceleration'
            ).value
        )
        self.max_angular_acceleration = float(
            self.get_parameter('constraints.max_angular_acceleration').value
        )
        self.max_linear_velocity = float(
            self.get_parameter('constraints.max_linear_velocity').value
        )
        self.max_angular_velocity = float(
            self.get_parameter('constraints.max_angular_velocity').value
        )

        self.lookahead_distance = float(
            self.get_parameter('controller.lookahead_distance').value
        )
        self.kP = float(self.get_parameter('controller.kP').value)
        self.kI = float(self.get_parameter('controller.kI').value)
        self.kD = float(self.get_parameter('controller.kD').value)

        # Creates variables to store current state
        self.path = None
        self.pose = None

        # Creates subscribers
        self.path_subscription = self.create_subscription(
            Path,
            self.path_map_topic,
            self.path_update_callback,
            10)
        self.position_subscription = self.create_subscription(
            Odometry,
            self.robot_position_topic,
            self.position_update_callback,
            10)

        # Creates publishers
        self.publisher = self.create_publisher(
            Twist,
            self.velocity_output_topic,
            10)

        self.timer = self.create_timer(1/self.controller_update_frequency, self.main_loop_callback)

    def path_update_callback(self, msg):
        self.path = msg

    def position_update_callback(self, msg):
        self.pose = msg

    def main_loop_callback(self):
        if self.path is None or self.pose is None:
            return
        if not self.path.poses:
            return

        robot_x = self.pose.pose.pose.position.x
        robot_y = self.pose.pose.pose.position.y

        goal = self.path.poses[-1].pose.position
        goal_distance = math.hypot(goal.x - robot_x, goal.y - robot_y)

        if goal_distance <= self.goal_tolerance:
            self.publisher.publish(Twist())
            return

        closest_point = min(
            self.path.poses,
            key=lambda point: ((point.pose.position.x - robot_x)**2
                               + (point.pose.position.y - robot_y)**2)
        )

        closest_index = self.path.poses.index(closest_point)

        lookahead_point = self.path.poses[-1]

        for point in self.path.poses[closest_index:]:
            distance = ((point.pose.position.x - robot_x)**2 + (point.pose.position.y - robot_y)**2)**0.5
            if distance >= self.lookahead_distance:
                lookahead_point = point
                break

        orientation = self.pose.pose.pose.orientation

        sin_yaw = 2 * (orientation.w * orientation.z
                    + orientation.x * orientation.y)

        cos_yaw = 1 - 2 * (orientation.y**2 + orientation.z**2)

        robot_yaw = math.atan2(sin_yaw, cos_yaw)

        dx = lookahead_point.pose.position.x - robot_x
        dy = lookahead_point.pose.position.y - robot_y

        local_x = dx * math.cos(robot_yaw) + dy * math.sin(robot_yaw)
        local_y = -dx * math.sin(robot_yaw) + dy * math.cos(robot_yaw)

        L_squared = local_x**2 + local_y**2

        if L_squared < 1e-6:
            self.publisher.publish(Twist())
            return

        curvature = 2 * local_y / L_squared

        if abs(curvature) > 1e-6:
            linear_velocity = min(
                self.max_linear_velocity,
                self.max_angular_velocity / abs(curvature),
                math.sqrt(self.max_sideways_linear_acceleration / abs(curvature))
            )
        else:
            linear_velocity = self.max_linear_velocity

        angular_velocity = linear_velocity * curvature

        current_linear_velocity = self.pose.twist.twist.linear.x
        current_angular_velocity = self.pose.twist.twist.angular.z

        dt = 1.0 / self.controller_update_frequency

        max_linear_change = self.max_forwards_linear_acceleration * dt
        max_angular_change = self.max_angular_acceleration * dt

        linear_velocity = max(
            current_linear_velocity - max_linear_change,
            min(linear_velocity, current_linear_velocity + max_linear_change)
        )

        angular_velocity = max(
            current_angular_velocity - max_angular_change,
            min(angular_velocity, current_angular_velocity + max_angular_change)
        )

        # velcoity command
        cmd = Twist()

        cmd.linear.x = linear_velocity
        cmd.angular.z = angular_velocity

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()