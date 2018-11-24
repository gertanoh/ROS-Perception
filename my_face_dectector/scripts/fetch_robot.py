#!/usr/bin/env python
import numpy
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry



# Move base using navigation stack
class MoveBasePublisher(object):

    def __init__(self):
        self._move_base_publisher = rospy.Publisher('/base_controller/command', 
            Twist, queue_size=1)
        self.odom_subs = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.actual_pose = Pose()
    
    def odom_callback(self,msg):
        self.actual_pose = msg.pose.pose
    
    def move_to(self, twist_object):
        self._move_base_publisher.publish(twist_object)
    
    def stop(self):
        twist_object = Twist()
        self.move_to(twist_object)
        
    def move_forwards_backwards(self,speed_ms,position_x, allowed_error=0.1):
        """
        distance_metres: positive is go forwards, negative go backwards
        """
        rate = rospy.Rate(5)
        move_twist = Twist()
        
        start_x = self.actual_pose.position.x
        direction_speed = numpy.sign(position_x - start_x)
        move_twist.linear.x = direction_speed*speed_ms
        
        
        in_place = False
        while not in_place:
            self.move_to(move_twist)
            x_actual = self.actual_pose.position.x
            print x_actual

            if  abs(position_x - x_actual) <= allowed_error:
                print "Reached position"
                break

            rate.sleep()
        self.stop()
        
def demo():
    # Create a node
    rospy.init_node("face_recognition_demo_node")


    
    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    
    move_base_pub = MoveBasePublisher()
    
    rospy.loginfo("Start Sequence complete")
    
    rospy.loginfo("Move to position...")
    move_base_pub.move_forwards_backwards(speed_ms=0.3,position_x=1.0, allowed_error=0.1)



    
if __name__ == "__main__":
    demo()