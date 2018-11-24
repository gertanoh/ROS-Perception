#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class LineFollower(object):
    
    def __init__(self):
        
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", 
            Image, self.camera_callback)
        
    def camera_callback(self, data):
        
        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data,
                desired_encoding="bgr8")
        except CvBridgeError as e :
            print(e)
            
        
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)
        
def main():
    
    line_follow_obj = LineFollower()
    rospy.init_node("line_following")
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')
        
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()
        