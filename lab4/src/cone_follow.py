#!/usr/bin/env python
import rospy
from lab4.msg import distance_angle
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped
import math
import numpy as np


class ConeFollower():
    def __init__(self):

        rospy.loginfo("PurePuruit node initialized")
        # Init subscribers and publishers
        self.sub = rospy.Subscriber("/controller_info", distance_angle,\
                self.PurePursuit, queue_size=1)
                
        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped, queue_size =1 )
                
	#declare variables
	self.steering = []

    def PurePursuit(self,msg):
	
	angle = msg.angle    #retrieve angle to reference point
	car_length = 0.46    #meters
	L_1 = msg.distance   #retrieve distance from reference point

	#set steering angle based on pure pursuit method covered in lecture
	steering_angle = math.atan(2 * car_length * math.sin(angle) / L_1)  

	#average 5 steering angle calculations to reduce "jerkiness" of controller
	self.steering.append(steering_angle) 
	if len(self.steering) > 5:
	    self.steering.pop(0)

	avg_angle = sum(self.steering) / len(self.steering) 
	
	#put together message and publish to high level mux
	new_msg = AckermannDriveStamped()
	new_msg.drive.steering_angle = avg_angle
	new_msg.drive.speed = 1.0
	self.pub.publish(new_msg)
	
	

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Cone_Follower_Node")

    # Init the node
    ConeFollower()

    # Don't let this script exit while ROS is still running
    rospy.spin()

