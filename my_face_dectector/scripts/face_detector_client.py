#!/usr/bin/env python

import rospy
from people_msgs.msg import PositionMeasurementArray


# Move base using navigation stack

class FaceDetectClient(object):
    
    def __init__(self):
        self.face_detector_sub = rospy.Subscriber("/face_detector/people_tracker_measurements_array",
                PositionMeasurementArray,
                self.face_detector_sub_callback)
        
        self.pos_measurement_array = PositionMeasurementArray()
    
    def face_detector_sub_callback(self, data):
        self.pos_measurement_array = data
        
        
def FaceDetectClient_start():
    
    rospy.init_node("face_detector_client_node")
    
    while not rospy.Time.now():
        pass
    
    face_detector_client = FaceDetectClient()
    rospy.spin()
    
if __name__ == "__main__":
    FaceDetectClient_start()