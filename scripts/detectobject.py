#!/usr/bin/env python
import rospy
import sys
import cv2
import time
import tf
from tf import TransformListener
import numpy as np
import math
from numpy.linalg import inv
import roslib
import baxter_interface
from baxter_interface import Limb
import image_geometry
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import Range
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from baxter_core_msgs.msg import CameraControl
from baxter_core_msgs.msg import CameraSettings
from lab_baxter_common.camera_control_helpers import CameraController
import struct
from operator import mul
from itertools import imap

class gripperCameraFeed(object):

    def __init__(self, limb='right'):

        self.limb = limb
        #self.cameraPath = "/cameras/%s_hand_camera/image" % (self.limb) 
        #self.rgb_image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.rgbKinectcallback, queue_size=1)
        #self.depth_registered_points_sub = rospy.Subscriber("/camera/depth_registered/points",PointCloud2,self.depthKinectcallback)
        self.imageSub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.colorFilter)
        self.bridge = CvBridge()
        self.minBlockArea = 500
        self.cXpixel, self.cYpixel = None, None 
    
    def colorFilter(self, data):
        #boundaries = [([100, 50, 50], [130, 255,255])] #blue filter
        boundaries = [([220/2, 0, 0],[130, 255, 255])] #blue filter for a bigger range of blues
        try:
            img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e: print(e)

        for (lower, upper) in boundaries:
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper) #mask for color
        blur = cv2.GaussianBlur(mask, (3, 3), 0)
        kernel = np.ones((5,5),np.uint8) #kernel slides through image as in 2D convolution
        opening = cv2.morphologyEx(img, cv2.MORPH_OPEN,kernel) #useful in reducing noise 
        output = cv2.bitwise_and(hsv, hsv, mask = blur) #apply mask to img
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        #find the contours

        for i , c in enumerate(contours):
            area = cv2.contourArea(c)
            M = cv2.moments(c) 

            if(area > self.minBlockArea):
                M = cv2.moments(c)
                cX = int(M["m10"]/M["m00"])
                cY = int(M["m01"]/M["m00"])

                self.cXpixel, self.cYpixel = cX, cY
                cv2.drawContours(img,contours,i,(0,0,255),3)
                #draw contour and circle on center of image 
                cv2.circle(img, (cX, cY), 5, (0, 0, 0), 2) 
                #draw black circle
                cv2.putText(img, "Center", (cX - 20, cY - 20), 
                #put text on circle 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                
        cv2.imshow("Color Detection", np.hstack([img, output]))
        cv2.waitKey(1) 


def main(args):
	ic = gripperCameraFeed()
	rospy.init_node("gripperCameraFeed", anonymous=True)
  	try:
  		rospy.spin()
  	except KeyboardInterrupt:
  		print("Shutting down")
  	cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)