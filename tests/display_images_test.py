#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from rviz_plugin_image_mesh.msg import RvizDisplayImages


def pub_image():

    rospy.init_node('rviz_display_images_test', anonymous=True)
    image_pub = rospy.Publisher("/display_images", RvizDisplayImages, queue_size=10)

    img = cv2.imread('./textures/bebop_drone.jpg',cv2.IMREAD_COLOR)
    img_msg = CvBridge().cv2_to_imgmsg(img, "bgr8")

    display_images = RvizDisplayImages()
    
    pose = Pose()
    
    pose.position.x =  1.0
    pose.position.y =  -0.5
    pose.position.z =  2.0

    # pose.position.x =  0.0
    # pose.position.y =  0.0
    # pose.position.z =  0.0

    pose.orientation.x = 0.70710678
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 0.70710678

    # pose.orientation.x = 0.0
    # pose.orientation.y = 0.0
    # pose.orientation.z = 0.0
    # pose.orientation.w = 1.0

    display_images.images = np.array([img_msg])
    display_images.poses = np.array([pose])
    display_images.scales = np.array([1.0, 1.0 * (332./590.)])

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        image_pub.publish(display_images)
        rate.sleep()

if __name__ == '__main__':

    try:
        pub_image()
    except rospy.ROSInterruptException:
        pass