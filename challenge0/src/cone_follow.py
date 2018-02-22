#!/usr/bin/env python
import rospy
from challenge0.msg import cone
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped
import math
import numpy as np


class ConeNavigator():
    def __init__(self):

        rospy.loginfo("PurePuruit node initialized")
        # Init subscribers and publishers
        self.sub = rospy.Subscriber("/controller_info", cone,\
                self.ConePass, queue_size=1)
                
        self.cone_pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
                AckermannDriveStamped, queue_size =1 )
                
	#declare variables
	self.steering = []
	self.path_distance = 0.5 #the distance we would like to pass the cone by

    """"Implements a Pure Pursuit algorithm to pass around the cone
    If the cone is green, it turns from the left?, if red, it turns from the right side.
    """
    def ConePass(self,msg):

	car_length = 0.46    #meters	
	angle = msg.angle    #retrieve angle to reference point
	L_1 = msg.distance   #retrieve distance from reference point
	cone_color = msg.color

	# account for turning from left/right of the cone
	if cone_color == 'green':
	    self.path_distance = -self.path_distance

	#Update point metrics to account for distance
	angle_diff = math.atan(self.path_distance/L_1) #angle added to account for passing around the cone
	angle += angle_diff
	L_1 = math.sqrt(L_1**2 + self.path_distance**2) #distance to the passing point

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
	new_msg.drive.speed = 0.5
	self.cone_pub.publish(new_msg)
	
	
if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Cone_Navigator_Node")

    # Init the node
    ConeNavigator()

    # Don't let this script exit while ROS is still running
    rospy.spin()

