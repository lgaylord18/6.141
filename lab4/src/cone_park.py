#!/usr/bin/env python
import rospy
from lab3.msg import distance_angle
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np


class ConeParker():
    def __init__(self):

        rospy.loginfo("Park node initialized")
        # Init subscribers and publishers
        self.sub = rospy.Subscriber("/controller_info", distance_angle,\
                self.SpeedController, queue_size=1)

	self.sub2 = rospy.Subscriber("/vesc/high_level/ackermann_cmd_mux/output",\
                AckermannDriveStamped, self.driveInfo, queue_size =1 )
	
                
        self.pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/safety",\
                AckermannDriveStamped, queue_size =1 )
                

	#declare variables

	self.Kp = 1                 #coefficient for proportional control
	self.desiredDistance = .5   #desired distance from reference point (meters)
	self.steeringAngle = 0      #desired angle from reference point

    #gets steering angle from high level mux, and adds it to the message to be published
    def driveInfo(self,msg):
	self.steeringAngle = msg.drive.steering_angle
	rospy.loginfo(self.steeringAngle)

    def SpeedController(self,msg):
	L_1 = msg.distance   #retrieve distance from reference point

	e =  L_1 - self.desiredDistance

	speed = self.Kp * e
	
	steeringAngle = self.steeringAngle*np.sign(speed)

	#put together message and publish to high level mux
	new_msg = AckermannDriveStamped()
	new_msg.drive.speed = speed
	new_msg.drive.steering_angle = steeringAngle
	self.pub.publish(new_msg)
	
	

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Cone_Park_Node")

    # Init the node
    ConeParker()

    # Don't let this script exit while ROS is still running
    rospy.spin()
