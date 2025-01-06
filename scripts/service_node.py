#!/usr/bin/env python3

import rospy
from assignment2_rt.srv import LastTarget, LastTargetResponse
from geometry_msgs.msg import Point

class TargetServiceNode:
    def _init_(self):
        # Initialize the node
        rospy.init_node('target_service_node')

        # Service to provide the last target
        self.service = rospy.Service('get_last_target', LastTarget, self.get_last_target_callback)

        # Store the last target position
        self.last_target = Point()
        rospy.loginfo("Service Node ready to provide the last target coordinates.")

    def set_last_target(self, x, y):
        """Update the last target with new coordinates."""
        self.last_target.x = x
        self.last_target.y = y

    def get_last_target_callback(self, request):
        """Callback to handle service requests."""
        return LastTargetResponse(
             x=self.last_target.x,
             y=self.last_target.y
         )


def main():
    # Create the service node
    node = TargetServiceNode()

    # Example: Set initial target coordinates
    node.set_last_target(2.0, 3.0)

    # Keep the node running
    rospy.spin()


if _name_ == '_main_':
    main()

