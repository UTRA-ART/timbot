#!/usr/bin/env python3

import threading
import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_srvs.srv import Empty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
w : increase forward speed by 0.1
a : increase leftward turn by 0.1rad/s 
s : increase backward speed by 0.1
d : increase rightward turn by 0.1rad/s 
p : give motor control to nav_stack (set to autonomous mode)

anything else : take control from nav_stack, stops rover 

CTRL-C to quit
"""

moveBindings = {
        'i': (1, 0, 0, 1),
    }

delta = 0.1
speedBindings = {
        'w': (delta, 0),
        's': (-delta, 0),
        'd': (0, -delta),
        'a': (0, delta),
    }


class PublishThread(threading.Thread):
    def __init__(self, node, rate, cmd_vel_topic):
        super(PublishThread, self).__init__()
        self.publisher = node.create_publisher(Twist, cmd_vel_topic, 1)
        self.node = node
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while rclpy.ok() and self.publisher.get_subscription_count() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to {}".format(
                    self.publisher.topic_name))
            rclpy.spin_once(self.node, timeout_sec=0.5)
            i += 1
            i = i % 5
        if not rclpy.ok():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, y, z, th, speed, turn):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.th = th
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        twist = Twist()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.th * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(twist)

        # Publish stop message when thread exits.
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)

        # Allow time for DDS to transmit the stop message before shutdown
        import time
        time.sleep(0.5)


def getKey(key_timeout):
    settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


def call_empty_service(node, service_name, timeout_sec=2.0):
    """Call an Empty service synchronously with a timeout."""
    client = node.create_client(Empty, service_name)
    if not client.wait_for_service(timeout_sec=timeout_sec):
        return False
    request = Empty.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    node.destroy_client(client)
    return future.result() is not None


def main(args=None):
    rclpy.init(args=args)
    settings = termios.tcgetattr(sys.stdin)

    node = Node('teleop_twist_keyboard')

    node.declare_parameter('speed', 0.0)
    node.declare_parameter('turn', 0.0)
    node.declare_parameter('repeat_rate', 0.001)
    node.declare_parameter('key_timeout', 0.0)
    node.declare_parameter('cmd_vel_topic', '/cmd_vel')

    speed = node.get_parameter('speed').value
    turn = node.get_parameter('turn').value
    repeat = node.get_parameter('repeat_rate').value
    key_timeout = node.get_parameter('key_timeout').value
    cmd_vel_topic = node.get_parameter('cmd_vel_topic').value
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(node, repeat, cmd_vel_topic)
    autonomous_mode = False
    mode_pub = node.create_publisher(Bool, '/pause_navigation', 1)

    x = -1
    y = 0
    z = 0
    th = 1
    status = 0
    top_vel = 2.2352  # m/s
    factor = 1 / .447 * 5280 * 12 / 10 / 3.1415 / 60 * 16
    b = -10.1124
    m = 640.449

    min_turn = -2 * top_vel / 0.445
    max_turn = 2 * top_vel / 0.445

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, y, z, th, speed, turn)

        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey(key_timeout)
            if key != 'p' and key != '' and autonomous_mode:
                node.get_logger().info(
                    "Autonomous mode set to false, Teleop control is active.")
                autonomous_mode = False
                mode_msg = Bool()
                mode_msg.data = not autonomous_mode
                mode_pub.publish(mode_msg)
            elif key == 'p':
                autonomous_mode = True
                mode_msg = Bool()
                mode_msg.data = not autonomous_mode
                mode_pub.publish(mode_msg)

                node.get_logger().info("Autonomous mode set to true.")
                if call_empty_service(node, '/move_base/clear_costmaps'):
                    node.get_logger().info("Cost map cleared.")
                else:
                    node.get_logger().info("Unable to clear cost map.")
                if call_empty_service(node, '/move_base/clear_unknown_space'):
                    node.get_logger().info("Unknown space cleared.")
                else:
                    node.get_logger().info("Unable to clear unknown space.")
                node.get_logger().info("Autonomous mode set to true.")
            else:
                mode_msg = Bool()
                mode_msg.data = not autonomous_mode
                mode_pub.publish(mode_msg)

            if key in speedBindings.keys():
                if speedBindings[key][1] != 0:  # case: changing angular vel
                    turn = turn + speedBindings[key][1]  # TODO: Write angular limits
                else:  # case: changing linear vel
                    if turn < 0:
                        speed = min(
                            max(speed + speedBindings[key][0],
                                (-2 * top_vel - turn * 0.89) / 2),
                            (2 * top_vel + turn * 0.89) / 2)
                    else:
                        speed = min(
                            max(speed + speedBindings[key][0],
                                (-2 * top_vel + turn * 0.89) / 2),
                            (2 * top_vel - turn * 0.89) / 2)

                speed = round(speed, 4)
                turn = round(turn, 4)

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == '' and autonomous_mode:
                    continue
                elif key != '':
                    x = -1
                    y = 0
                    z = 0
                    th = 1
                    speed = 0
                    turn = 0
                if (key == '\x03'):
                    break

            pub_thread.update(x, y, z, th, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        node.destroy_node()
        rclpy.shutdown()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
