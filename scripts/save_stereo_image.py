#!/usr/bin/env python

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

# Instantiate CvBridge
bridge = CvBridge()
left=False
right=False
pid = 0

def left_image_callback(msg):
    #print("Received a left image!")
    global left
    if left==True:
	try:
	    # Convert your ROS Image message to OpenCV2
	    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
	except CvBridgeError, e:
	    print(e)
	    left=False
	else:
	    # Save your OpenCV2 image as a jpeg 
	    print("going to save a left image: ",msg.header.stamp)
	    cv2.imwrite('/home/csc104/jym/catkin_ws/src/pylon_camera/image/left_%d.png' % pid, cv2_img)
	    left=False

def right_image_callback(msg):
    #print("Received a right image!")
    global right
    if right==True:
	try:
	    # Convert your ROS Image message to OpenCV2
	    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
	except CvBridgeError, e:
	    print(e)
	    right=False
	else:
	    # Save your OpenCV2 image as a jpeg 
	    print("going to save a right image: ",msg.header.stamp)
	    cv2.imwrite('/home/csc104/jym/catkin_ws/src/pylon_camera/image/right_%d.png' % pid, cv2_img)
	    right=False

def key_callback(msg):
    global left
    global right
    global pid
    print("Received a key to save image!")
    left=True
    right=True
    pid += 1
	

def main():
    rospy.init_node('image_listener')
    # Define your image topic
    left_image_topic = "/pylon/left/camera_left/image_raw"
    right_image_topic = "/pylon/right/camera_right/image_raw"
    key_topic = "/key"
    # Set up your subscriber and define its callback
    rospy.Subscriber(left_image_topic, Image, left_image_callback)
    rospy.Subscriber(right_image_topic, Image, right_image_callback)
    rospy.Subscriber(key_topic, Bool, key_callback)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()
