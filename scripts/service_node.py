#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import Trigger  # Modify this if using a custom service
from geometry_msgs.msg import Pose  # For target position

class TargetServiceNode(Node):
    def __init__(self):
        super().__init__('target_service_node')
        self.srv = self.create_service(Trigger, 'get_last_target', self.get_last_target_callback)
        self.last_target = Pose()
        self.get_logger().info("Service Node ready to provide the last target coordinates.")

    def set_last_target(self, x, y):
        """ Update the last target with new coordinates. """
        self.last_target.position.x = x
        self.last_target.position.y = y

    def get_last_target_callback(self, request, response):
        response.success = True
        response.message = (
            f"Last target coordinates: x={self.last_target.position.x}, "
            f"y={self.last_target.position.y}"
        )
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TargetServiceNode()
    node.set_last_target(2.0, 3.0)  # Example target coordinates
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

