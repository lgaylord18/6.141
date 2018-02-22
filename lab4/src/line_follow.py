#!/usr/bin/env python
import rospy
from lab4.msg import distance_angle
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped
import math
import numpy as np
from threading import Thread


class LineFollower():
    def __init__(self):

        rospy.loginfo("PurePuruit node initialized")
        # Init subscribers and publishers
        self.sub = rospy.Subscriber("/controller_info_low", distance_angle,\
                self.processLow, queue_size=1)

        self.sub = rospy.Subscriber("/controller_info_high", distance_angle,\
                self.processHigh, queue_size=1)
                
        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped, queue_size =1 )
                
	

        #declare variables
	self.low_steering = []	#list of steering angles retreived from p control
	self.high_steering = []	 # list of steering angles retreived from pure pursuit

	self.Ka = .5     #gain coefficient
	self.Kd = 4      #derivative coefficient

	self.car_length = 0.175 #distance in M wheels to the center of mass

	self.speed = 2

	self.old_angle = 0
	self.desired_angle = 0

	self.weight = 1		#gain for weighted average for steering angles from two control methods

	self.drive_thread = Thread(target=self.finalSteering)	#run finalSteering funciton on a thread
	self.drive_thread.start()

    #retrieve list of steering angles from pd controller
    def processLow(self, msg):
	self.proportional(msg)

    #retrieve list of steering angles from pure pursuit controller
    def processHigh(self, msg):
	self.PurePursuit(msg)


    def finalSteering(self):

	while not rospy.is_shutdown():
		if len(self.low_steering)==0 or len(self.high_steering)==0:
		    pass
		else:
		    #get the steering angles by averaging the last five of both low and high
		    low_steering = sum(self.low_steering) / len(self.low_steering)
		    high_steering = sum(self.high_steering)/len(self.high_steering)

		    #average the steering angles using weight
		    self.steering = (low_steering * self.weight + high_steering)/(self.weight+1)

		    #put together message and publish
        	    new_msg = AckermannDriveStamped()
        	    new_msg.drive.steering_angle = self.steering
        	    new_msg.drive.speed = self.speed
        	    self.pub.publish(new_msg)

		    #run this at 20hz
		    rospy.sleep(1.0/20.0)


    #pd controller 
    def proportional(self,msg):
	angle = msg.angle
	error = angle - self.desired_angle
	derivative = self.old_angle - angle
	steering_angle = error*self.Ka - self.Kd*derivative
	self.old_angle = angle

        #average 5 steering angle calculations to reduce "jerkiness" of controller
        self.low_steering.append(steering_angle) 
        if len(self.low_steering) > 5:
            self.low_steering.pop(0)

    #pure pursuit controller
    def PurePursuit(self,msg):
        
        angle = msg.angle
        L_1 = msg.distance   #retrieve distance from reference point

        steering_angle = math.atan(2 * self.car_length * math.sin(angle) / L_1)

        #average 5 steering angle calculations to reduce "jerkiness" of controller
        self.high_steering.append(steering_angle) 
        if len(self.high_steering) > 5:
            self.high_steering.pop(0) 
        
        

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Line_Follower_Node")

    # Init the node
    LineFollower()

    # Don't let this script exit while ROS is still running
    rospy.spin()
