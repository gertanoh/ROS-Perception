#!/usr/bin/env python   
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState


"""
I use the topic /mira/joint_states to get the current yaw value,
the topic /mira/yaw_joint_position_controller/command
to publish the desired yaw
"""


pub_mira = None
current_yaw = None
expected_yaw = 0.0

def vel_callback(data):
    global current_yaw
    global pub_mira
    global expected_yaw
    
    expected_yaw = current_yaw + data.angular.z
    pub_mira.publish(expected_yaw)
    
def joint_callback(data):
    global current_yaw
    current_yaw = data.position[2]
    
def run():
    
    
    rospy.init_node("track_move_pkg");
    
    global current_yaw
    global expected_yaw
    global pub_mira
    
    pub_mira = rospy.Publisher("/mira/yaw_joint_position_controller/command",
        Float64, queue_size=1)
    rospy.Subscriber("/mira/commands/velocity", Twist, vel_callback)
    rospy.Subscriber("/mira/joint_states", JointState, joint_callback)

    
    
    # ensure that we have a starting yaw value
    while current_yaw is None:
        try:
            joint_data = rospy.wait_for_message("/mira/joint_states", 
                JointState, timeout=4)
            current_yaw = joint_data.position[2]
        except:
            rospy.loginfo("Time out for first value of yaw")
            pass
    while not rospy.is_shutdown():
        
        # rospy.loginfo("Running")
        # rospy.loginfo("current_yaw : " + str(current_yaw))
        # rospy.loginfo("expected_yaw : " + str(expected_yaw))
        rospy.sleep(.1)
        
if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass