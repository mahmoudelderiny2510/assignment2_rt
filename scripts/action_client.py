#!/usr/bin/env python3

import rospy
import actionlib
from assignment2_rt.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String
from assignment2_rt.srv import LastTarget, LastTargetResponse


# Global variables
robot_position = Point()
last_target = Point()  # Store the last target provided by the user


def odom_callback(data):
    """Callback function to update the robot's position."""
    global robot_position
    robot_position = data.pose.pose.position


def send_goal(client, x, y):
    """Send a goal to the action server."""
    global last_target
    last_target.x = x  # Update last target
    last_target.y = y
    goal = PlanningGoal()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    client.send_goal(goal)
    rospy.loginfo(f"Goal sent: x={x}, y={y}")


def cancel_goal(client):
    """Cancel the current goal."""
    client.cancel_goal()
    rospy.loginfo("Goal cancelled.")


def handle_get_last_target(req):
    """Service callback to return the last target."""
    rospy.loginfo(f"Returning last target: x={last_target.x}, y={last_target.y}")
    return LastTargetResponse(x=last_target.x, y=last_target.y)


def main():
    rospy.init_node('action_client_node')

    # Action client setup
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Action server is ready.")

    # Subscriber to /odom
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Publisher for custom feedback (optional)
    feedback_pub = rospy.Publisher('/reaching_goal/feedback', String, queue_size=10)

    # Advertise the service to provide the last target
    rospy.Service('get_last_target', LastTarget, handle_get_last_target)
    rospy.loginfo("Service 'get_last_target' is ready.")

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        user_input = input("Enter target (x, y) or 'cancel': ")

        if user_input.lower() == 'cancel':
            cancel_goal(client)
        else:
            try:
                x, y = map(float, user_input.split(','))
                send_goal(client, x, y)

                # Monitor the goal's progress
                while not client.wait_for_result(timeout=rospy.Duration(1.0)):
                    feedback_msg = f"Current position: x={robot_position.x:.2f}, y={robot_position.y:.2f}"
                    rospy.loginfo(feedback_msg)
                    feedback_pub.publish(feedback_msg)

                rospy.loginfo("Goal reached!")
            except ValueError:
                rospy.logerr("Invalid input. Please enter valid x, y coordinates or 'cancel'.")

        rate.sleep()


if _name_ == '_main_':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

