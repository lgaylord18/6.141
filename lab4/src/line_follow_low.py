#!/usr/bin/env python
import rospy
from lab4.msg import distance_angle
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped
import math
import numpy as np



class LineFollower():
    def __init__(self):

        rospy.loginfo("PurePuruit node initialized")
        # Init subscribers and publishers
        self.sub = rospy.Subscriber("/controller_info", distance_angle,\
                self.proportional, queue_size=1)
                
        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped, queue_size =1 )
                
	
        #declare variables
        self.steering = []
	
	self.speed = 2.5

	self.Ka = .5      #gain coefficient
	self.Kd = 2       #derivative coefficient

	self.old_angle = None
	self.desired_angle = 0
	
    #pd controller	
    def proportional(self,msg):
	angle = msg.angle                  #get angle from message from controller info
	if self.old_angle == None:
	    self.old_angle = angle         #after init old angle set to first angle
	error = angle-self.desired_angle
	derivative =  angle - self.old_angle
	
	#set steering angle using pd control
	steering_angle = error*self.Ka - derivative*self.Kd       
	self.old_angle = angle



        #average 5 steering angle calculations to reduce "jerkiness" of controller
        self.steering.append(steering_angle) 
        if len(self.steering) > 5:
            self.steering.pop(0)

        avg_angle = sum(self.steering) / len(self.steering) 
        

        #put together message and publish to high level mux
        new_msg = AckermannDriveStamped()
        new_msg.drive.steering_angle = avg_angle
        new_msg.drive.speed = self.speed
        self.pub.publish(new_msg)
        
        

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Line_Follower_Node")

    # Init the node
    LineFollower()

    # Don't let this script exit while ROS is still running
    rospy.spin()

