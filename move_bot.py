#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

# Initialize ROS node
rospy.init_node('turtlebot_turn_controller')

# Publisher for TurtleBot velocity commands
velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

# Define the angular velocity and time to rotate 45 degrees
angular_speed = 0.5  # radians per second (tune this value based on your robot)
rotation_duration = (45 * (3.14159 / 180)) / angular_speed  # time to rotate 45 degrees
rotation_duration += 0.1

# Flag to track if the robot has already rotated
has_rotated = False

# Initialize Twist message
twist = Twist()

# Callback function for /pointing_hand
def pointing_hand_callback(msg):
    global has_rotated
    if not has_rotated:
        rospy.loginfo("Received pointing direction: %d", msg.data)
        if msg.data == 1:
            # Turn the TurtleBot clockwise
            rospy.loginfo("Turning clockwise")
            rotate_turtlebot(-angular_speed, rotation_duration)
        elif msg.data == 2:
            # Turn the TurtleBot anticlockwise
            rospy.loginfo("Turning anticlockwise")
            rotate_turtlebot(angular_speed, rotation_duration)
        has_rotated = True  # Set the flag to True after initiating the rotation

def rotate_turtlebot(speed, duration):
    twist.angular.z = speed
    start_time = rospy.Time.now().to_sec()
    rate = rospy.Rate(10)  # Publish at 10 Hz

    while rospy.Time.now().to_sec() - start_time < duration:
        velocity_pub.publish(twist)
        rate.sleep()

    # Stop the TurtleBot after the rotation
    twist.angular.z = 0
    velocity_pub.publish(twist)
    rospy.loginfo("Rotation completed")

# Subscribe to the /pointing_hand topic
rospy.Subscriber('/pointing_hand', Int32, pointing_hand_callback)

# Run until shutdown (to keep the node alive until it's intentionally stopped)
rospy.spin()
