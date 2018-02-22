#!/usr/bin/env python
import rospy
import math
from lab3.msg import distance_angle
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np

"""
Author: Ariel Anders
This program tells the robot to follow a wall.

Edited by: Winter Guerra
"""

class WallFollower():
    def __init__(self):

        rospy.loginfo("PD node initialized")
        # Init subscribers and publishers
        self.sub = rospy.Subscriber("/controller_info", distance_angle,\
                self.PD, queue_size=1)
                
        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped, queue_size =1 )
                

	#Initialize all the constants
	self.v = .25 #velocity
	self.l = .2 #distance from center of mass to steering axis of robot
	self.t_increment = .025 #40 hz
	self.komega = self.l/self.v #way of converting omega to steering angle 
	self.Ka = 1.0
	self.Kd = self.Ka/self.v/4.0 #found using PID
	self.desired_distance = 1

	#using these two to reduce noise in the steering angle
	self.steering = []
	self.WINDOW_SIZE = 5
	


    def PD(self,msg):
	#rospy.loginfo("distance = %.4f and angle = %.4f" % (msg.distance,msg.angle))
	
	#get data from wall-scanner
	current_distance = msg.distance
	current_angle = msg.angle

	#find the error in the distance and multiply by kd to find an angle to the wall that we want
	desired_angle = (self.desired_distance - current_distance)*self.Kd
	
	#Dont turn more than 30degrees left or right
	if desired_angle<math.radians(-30): desired_angle = math.radians(-30)
	if desired_angle>math.radians(30): desired_angle = math.radians(30)

	#find the error in the distance and multiply by ka to find an omega
	desired_omega = (desired_angle-current_angle)*self.Ka
	
	#given an omega use our komega to get a steering angle (used geometric derivates to solve for komega)
	steering_angle = self.komega*desired_omega

	#take a rolling averages of steering_angle using self.WINDOWSIZE and averaging inorder to reduce noise
	self.steering.append(steering_angle)
	if len(self.steering) > self.WINDOW_SIZE:
		self.steering.pop(0)
	avg_angle = sum(self.steering) / len(self.steering)
	
	#send the message to the highlevel mux
	new_msg = AckermannDriveStamped()
	new_msg.drive.steering_angle = avg_angle
	new_msg.drive.speed = self.v
	self.pub.publish(new_msg)
	
	

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Wall_Follower_Node")

    # Init the node
    WallFollower()

    # Don't let this script exit while ROS is still running
    rospy.spin()

