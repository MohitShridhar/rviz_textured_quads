#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

import copy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

from rviz_plugin_image_mesh.msg import TexturedQuad, TexturedQuadArray


def pub_image():

    rospy.init_node('rviz_display_image_test', anonymous=True)
    image_pub = rospy.Publisher("/textured_quads", TexturedQuadArray, queue_size=10)

    img = cv2.imread('./textures/bebop_drone.jpg',cv2.IMREAD_COLOR)
    img_msg = CvBridge().cv2_to_imgmsg(img, "bgr8")

    img2 = cv2.imread('./textures/Decal.png',cv2.IMREAD_COLOR)
    img_msg2 = CvBridge().cv2_to_imgmsg(img2, "bgr8")

    display_image = TexturedQuad()
    
    pose = Pose()
    
    pose.position.x =  -1.2
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

    display_image.image = img_msg
    display_image.pose = pose
    display_image.width = 1.0  
    display_image.height = (1.0 * img_msg.height)/img_msg.width

    second_image = copy.deepcopy(display_image)
    second_image.image = img_msg2
    second_image.width = 1.0
    second_image.height = 1.0
    second_image.pose.position.x = 0.2
    second_image.pose.position.y = -0.3
    second_image.pose.position.z = 2.0

    second_image.pose.orientation.x = 0.0
    second_image.pose.orientation.y = 0.70710678
    second_image.pose.orientation.z = 0.0
    second_image.pose.orientation.w = 0.70710678

    display_images = TexturedQuadArray()
    display_images = np.array([display_image])

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        image_pub.publish(display_images)
        rate.sleep()

if __name__ == '__main__':

    try:
        pub_image()
    except rospy.ROSInterruptException:
        pass