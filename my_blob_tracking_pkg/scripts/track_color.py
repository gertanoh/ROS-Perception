#!/usr/bin/env python   
import rospy
from geometry_msgs.msg import Twist, Vector3
from cmvision.msg import Blobs, Blob
from random import randint


# pose is not relevant here
# the image is divided into three spaces in width direction
# left, center and right
# center is between 190 and 210
x_pose = 0
turn = 0.0

# 0 for center, -1 for left and 1 for right
prev_pos = None;

def callback(data):
    
    global prev_pos
    global x_pose
    global turn
    
    if(len(data.blobs)):
        for blob in data.blobs:
            if blob.name == "RedBall":
                x_pose = blob.x
                # .2 speed was too fast and resulted in jerking
                
                if x_pose > 220:
                    # turn right
                    prev_pos = 1
                    turn = -0.1
                    
                elif x_pose < 180:
                    prev_pos = -1
                    turn = 0.1
                    
                elif x_pose > 180 and x_pose < 220:
                    prev_pos = 0
                    turn = 0.0
    
    else:
        # No blob found in the image
        # search pattern
        if prev_pos is None or prev_pos == 0:
            # the red ball has not been seen yet
            r_number = randint(0, 1)
            if r_number == 0:
                # search by right
                turn = -0.3
            else:
                turn = -0.3
            
        elif prev_pos == 1:
            # probably swerve to the far right
            turn = -0.3
            
        elif prev_pos == -1:
            turn = 0.3
            
        
        
        
    
def run():
    
    # Init node
    rospy.init_node("track_blob_pkg");
    
    # Publiish twist velocity to mira robot
    pub = rospy.Publisher("/mira/commands/velocity", Twist, queue_size = 1)
    
    # subscribe to cmvision
    sub = rospy.Subscriber("/blobs", Blobs, callback)
    
    
    # run while ros is ok
    twist = Twist()
    
    global turn
    global x_pose
    
    while not rospy.is_shutdown():
        
        # rospy.loginfo("pose : " +str(x_pose) + ", command : " + str(turn) )
        
        twist = Twist(Vector3(0,0,0), Vector3(0,0,turn))
        turn = 0.0
        
        pub.publish(twist)
        rospy.sleep(.1)

        
        
if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass