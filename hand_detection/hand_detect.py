#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
import cv2
import mediapipe as mp
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError

# Initialize ROS node
rospy.init_node('hand_detection_node')

# ROS Publisher for hand pointing detection
pointing_pub = rospy.Publisher('/pointing_hand', Int32, queue_size=10)

# Initialize MediaPipe components for pose detection
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

# Define drawing specifications for landmarks and connections
drawing_spec = mp_drawing.DrawingSpec(thickness=2, circle_radius=2, color=(0, 255, 0))

# Bridge to convert ROS Image messages to OpenCV images
bridge = CvBridge()

def calculate_angle(landmark1, landmark2, landmark3):
    a = np.array([landmark1.x, landmark1.y])
    b = np.array([landmark2.x, landmark2.y])
    c = np.array([landmark3.x, landmark3.y])
    ba = a - b
    bc = c - b
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)
    return np.degrees(angle)

def is_landmark_visible(landmark):
    return landmark.visibility > 0.5

# Callback function to handle image processing
def image_callback(msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    except CvBridgeError as e:
        rospy.logerr(f"Failed to convert image: {str(e)}")
        return

    # Process hand and pose data from the image
    pose_results = mp_pose.process(rgb_image)
    pointing_detected = False

    if pose_results.pose_landmarks:
        mp_drawing.draw_landmarks(cv_image, pose_results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS)
        
        # Right arm
        right_elbow = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_ELBOW]
        right_shoulder = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_SHOULDER]
        right_hip = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_HIP]
        right_wrist = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.RIGHT_WRIST]

        # Left arm
        left_elbow = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_ELBOW]
        left_shoulder = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_SHOULDER]
        left_hip = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_HIP]
        left_wrist = pose_results.pose_landmarks.landmark[mp.solutions.pose.PoseLandmark.LEFT_WRIST]

        # Check if the right arm is visible and pointing
        if is_landmark_visible(right_elbow) and is_landmark_visible(right_shoulder) and is_landmark_visible(right_hip):
            right_angle = calculate_angle(right_elbow, right_shoulder, right_hip)
            right_elbow_bend_angle = calculate_angle(right_shoulder, right_elbow, right_wrist)
            if right_angle > 18 and right_elbow_bend_angle > 155:  # Consider straight if angle is above 150 degrees
                rospy.loginfo("Right arm pointing detected with angle: {:.2f}".format(right_angle))
                cv2.putText(cv_image, "Right Arm Pointing Detected", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                pointing_detected = True
                pointing_pub.publish(2)  # Publish 2 for right hand

        # Check if the left arm is visible and pointing
        if is_landmark_visible(left_elbow) and is_landmark_visible(left_shoulder) and is_landmark_visible(left_hip):
            left_angle = calculate_angle(left_elbow, left_shoulder, left_hip)
            left_elbow_bend_angle = calculate_angle(left_shoulder, left_elbow, left_wrist)
            if left_angle > 18 and left_elbow_bend_angle > 155:  # Consider straight if angle is above 150 degrees
                rospy.loginfo("Left arm pointing detected with angle: {:.2f}".format(left_angle))
                cv2.putText(cv_image, "Left Arm Pointing Detected", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                pointing_detected = True
                pointing_pub.publish(1)  # Publish 1 for left hand

    cv2.imshow('Pose Detection', cv_image)
    cv2.waitKey(1)

# Cleanup function
def cleanup():
    cv2.destroyAllWindows()

# Subscribe to the RGB image topic
rospy.Subscriber('/camera/rgb/image_reduced', Image, image_callback)

# Register cleanup function to be called on node shutdown
rospy.on_shutdown(cleanup)

# Keep the node alive
rospy.spin()
