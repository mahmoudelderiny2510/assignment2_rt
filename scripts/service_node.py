#!/usr/bin/env python3

"""Service Node for providing the last target coordinates.

This node maintains the last target position requested by the user
and provides it through a ROS service when requested.
"""

import rospy
from assignment2_rt.srv import LastTarget, LastTargetResponse
from geometry_msgs.msg import Point


class TargetServiceNode:
    """A ROS node that provides the last target coordinates through a service.
    
    Attributes:
        last_target (Point): Stores the last target coordinates (x, y).
    """
    
    def __init__(self):
        """Initialize the TargetServiceNode.
        
        Sets up the ROS node and creates the 'get_last_target' service.
        """
        # Initialize the node
        rospy.init_node('target_service_node')

        # Service to provide the last target
        self.service = rospy.Service('get_last_target', LastTarget, self.get_last_target_callback)

        # Store the last target position
        self.last_target = Point()
        rospy.loginfo("Service Node ready to provide the last target coordinates.")

    def set_last_target(self, x, y):
        """Update the last target with new coordinates.
        
        Args:
            x (float): The x-coordinate of the new target.
            y (float): The y-coordinate of the new target.
        """
        self.last_target.x = x
        self.last_target.y = y

    def get_last_target_callback(self, request):
        """Callback to handle service requests.
        
        Args:
            request (LastTargetRequest): The service request (empty in this case).
        
        Returns:
            LastTargetResponse: Contains the x and y coordinates of the last target.
        """
        return LastTargetResponse(
             x=self.last_target.x,
             y=self.last_target.y
         )


def main():
    """Main function to create and run the TargetServiceNode.
    
    Creates an instance of TargetServiceNode and sets an initial target.
    """
    # Create the service node
    node = TargetServiceNode()

    # Example: Set initial target coordinates
    node.set_last_target(2.0, 3.0)

    # Keep the node running
    rospy.spin()


if __name__ == '__main__':
    main()
