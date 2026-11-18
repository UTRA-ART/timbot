#!/usr/bin/env python3
"""Republish the Nav2 costmaps slightly above the map, for Foxglove only.

The costmaps and Cartographer's /map all lie at z=0, so Foxglove draws them in the
same plane and they flicker (z-fighting). Foxglove has no per-grid height setting,
so this republishes each costmap with its origin raised a few cm:
    /global_costmap/costmap -> /foxglove/global_costmap   (+2 cm)
    /local_costmap/costmap  -> /foxglove/local_costmap    (+4 cm)
Nav2 is untouched. A costmap is only subscribed to while something (Foxglove)
subscribes to its lifted copy, so it costs nothing when hidden.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

LIFTS = {'/global_costmap/costmap': ('/foxglove/global_costmap', 0.02),
         '/local_costmap/costmap': ('/foxglove/local_costmap', 0.04)}
QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)


class CostmapLift(Node):
    def __init__(self):
        super().__init__('costmap_lift')
        self.routes = {}   # source -> [publisher, lift, subscription or None]
        for src, (dst, lift) in LIFTS.items():
            self.routes[src] = [self.create_publisher(OccupancyGrid, dst, QOS), lift, None]
        self.create_timer(1.0, self.update_subscriptions)

    def update_subscriptions(self):
        for src, route in self.routes.items():
            pub, lift, sub = route
            watched = pub.get_subscription_count() > 0
            if watched and sub is None:
                route[2] = self.create_subscription(
                    OccupancyGrid, src, lambda m, p=pub, z=lift: self.relay(m, p, z), QOS)
            elif not watched and sub is not None:
                self.destroy_subscription(sub)
                route[2] = None

    @staticmethod
    def relay(msg, pub, lift):
        msg.info.origin.position.z += lift
        pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(CostmapLift())


if __name__ == '__main__':
    main()
