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
                self.proPortional, queue_size=1)
                
        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped, queue_size =1 )
                

	#declare variables
	self.steering = []              #list of steering angles (to be averaged)

	self.Ka = 1                     #gain coefficient
	self.Kd = .5                    #derivative coefficient

	self.oldTheta = None           #old angle to reference point
	self.desiredAngle = 0          #desired angle to reference point, which we set to 0, and doesn't change

    def proPortional(self,msg):
	if self.oldTheta == None:
	    self.oldTheta = msg.angle       #when first init, old theta is set to current theta
	angle = msg.angle                   #get angle from msg
	error = angle-self.desiredAngle
	derivative = self.oldTheta-angle
	self.oldTheta = angle    
	

	steering_angle = error*self.Ka - derivative*self.Kd  #set steering angle using pd control

	#average 5 steering angle calculations to reduce "jerkiness" of controller
	self.steering.append(steering_angle) 
	if len(self.steering) > 5:
	    self.steering.pop(0)

	avg_angle = sum(self.steering) / len(self.steering) 
	rospy.loginfo(self.steering)
	
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

