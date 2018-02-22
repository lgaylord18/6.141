#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped


class OpenloopCircle():
	
    def __init__(self):
        rospy.loginfo("Open Loop Control node initialized")
	#init publisher
        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/default",\
                AckermannDriveStamped, queue_size =1 )
	
        self.steering_angle = .225  #steering angle (radians)
        self.speed = 0.65           #speed
        
        while True:
                self.drive()
               	#rospy.sleep(0.05)
    
    #publish speed and steering angle
    def drive(self):
        new_msg = AckermannDriveStamped()
        new_msg.drive.speed = self.speed
        new_msg.drive.steering_angle = self.steering_angle
	self.pub.publish(new_msg)

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Openloop_Circle_Node")

    # Init the node
    OpenloopCircle()

    # Don't let this script exit while ROS is still running
    rospy.spin()
