#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image

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
    display_images.images = np.array([img_msg])

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        image_pub.publish(display_images)
        rate.sleep()

if __name__ == '__main__':

    try:
        pub_image()
    except rospy.ROSInterruptException:
        pass