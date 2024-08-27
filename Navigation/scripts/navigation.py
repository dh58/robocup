#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Point, Quaternion, Pose  # Added Pose import
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
        # Initialize state variables
        self.start = 1
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server")
        
        rospy.loginfo("Ready to go")
        rospy.sleep(1)
    
        # Define the target location (point A)
        A_x = -2.74
        A_y = -0.0896
        A_theta = 0
        
        quaternion = quaternion_from_euler(0.0, 0.0, A_theta)
        self.target_location = Pose(Point(A_x, A_y, 0.0), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        
        self.goal = MoveBaseGoal()
        rospy.loginfo("Starting navigation test")

        while not rospy.is_shutdown():
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()
            
            # Robot will go to point A
            if self.start == 1:
                rospy.loginfo("Going to point A")
                rospy.sleep(2)
                self.goal.target_pose.pose = self.target_location
                self.move_base.send_goal(self.goal)
                waiting = self.move_base.wait_for_result(rospy.Duration(300))

                # Check if the goal was reached
                if waiting and self.move_base.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reached point A")
                    rospy.sleep(2)
                    self.start = 2  # Stop after reaching point A
            
            rospy.Rate(5).sleep()

    def cleanup(self):
        rospy.loginfo("Shutting down navigation....")
        self.move_base.cancel_goal()

if __name__ == "__main__":
    rospy.init_node('navi_point', anonymous=True)
    try:
        NavToPoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
