#!/usr/bin/env python

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal  # Replace with your action name
from nav_msgs.msg import Odometry
from custom_msgs.msg import PositionVelocity  # Replace with your custom message
from geometry_msgs.msg import Twist

# Global variables
robot_position = None
robot_velocity = None


def odom_callback(data):
    global robot_position, robot_velocity

    # Extract position
    robot_position = data.pose.pose.position

    # Extract velocity
    robot_velocity = data.twist.twist


def publish_position_velocity(pub):
    global robot_position, robot_velocity

    if robot_position and robot_velocity:
        msg = PositionVelocity()
        msg.x = robot_position.x
        msg.y = robot_position.y
        msg.vel_x = robot_velocity.linear.x
        msg.vel_z = robot_velocity.angular.z
        pub.publish(msg)


def main():
    rospy.init_node('action_client_node')

    # Create action client
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Action server available.")

    # Set up publisher for robot position and velocity
    position_velocity_pub = rospy.Publisher('/robot_position_velocity', PositionVelocity, queue_size=10)

    # Subscribe to /odom topic for robot's position and velocity
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Rate for publishing data
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # Prompt user for input
        user_input = input("Enter target (x, y) or 'cancel': ")

        if user_input.lower() == 'cancel':
            rospy.loginfo("Cancelling the goal...")
            client.cancel_goal()
        else:
            try:
                # Parse user input
                x, y = map(float, user_input.split(','))
                rospy.loginfo(f"Sending goal: x={x}, y={y}")

                # Create and send goal
                goal = PlanningGoal()
                goal.target_pose.pose.position.x = x
                goal.target_pose.pose.position.y = y
                client.send_goal(goal)

                # Monitor progress
                while not client.wait_for_result(timeout=rospy.Duration(1.0)):
                    rospy.loginfo("Waiting for the goal to be reached...")
                
                rospy.loginfo("Goal reached!")
            except ValueError:
                rospy.logerr("Invalid input. Enter x, y coordinates or 'cancel'.")

        # Publish position and velocity
        publish_position_velocity(position_velocity_pub)

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

