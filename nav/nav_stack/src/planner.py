import rclpy
from rclpy.node import Node
# ADD NECCESSARY IMPORTS HERE

class NavPlanner(Node):
    def __init__(self):
        super().__init__('nav_planner')

        # Gets all the parameters from config specified in launch file
        

        # TO ADD HERE

    # TO ADD HERE

def main(args=None):
    rclpy.init(args=args)
    node = NavPlanner()
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