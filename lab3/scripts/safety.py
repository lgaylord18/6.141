#!/usr/bin/env python
import rospy
import scipy.ndimage as sc
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped

import numpy as np

"""
Author: Ariel Anders
This program implements a simple safety node based on laser
scan data.

Edited by: Winter Guerra

# Single scan from a planar laser range-finder
# This is the ROS message structure

Header header
# stamp: The acquisition time of the first ray in the scan.
# frame_id: The laser is assumed to spin around the positive Z axis
# (counterclockwise, if Z is up) with the zero angle forward along the x axis

float32 angle_min # start angle of the scan [rad]
float32 angle_max # end angle of the scan [rad]
float32 angle_increment # angular distance between measurements [rad]

float32 time_increment # time between measurements [seconds] - if your scanner
# is moving, this will be used in interpolating position of 3d points
float32 scan_time # time between scans [seconds]

float32 range_min # minimum range value [m]
float32 range_max # maximum range value [m]

float32[] ranges # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities # intensity data [device-specific units]. If your
# device does not provide intensities, please leave the array empty.

"""

class Safety():
    def __init__(self):

        # Init subscribers and publishers
        self.sub = rospy.Subscriber("/scan", LaserScan,\
                self.lidarCB, queue_size=1)

	self.sub2 = rospy.Subscriber("/vesc/high_level/ackermann_cmd_mux/output", AckermannDriveStamped,\
		self.driveInfo, queue_size=1)
                
        self.pub = rospy.Publisher("/vesc/low_level/ackermann_cmd_mux/input/safety",\
                AckermannDriveStamped, queue_size =1 )

	self.LOOKING_RANGE = 30 #we look + and - from in front of us (this is in degrees)
	self.DANGER_DISTANCE = .25 #max dist to object to cause robot to stop
	self.CAUTION_DISTANCE = 1 #max dist to object to cause robot to slowdown
        self.SPEED = .5
	self.ANGLE = 0

        rospy.loginfo("Safety node initialized")


    def driveInfo(self,msg):
	self.SPEED = msg.drive.speed
	self.ANGLE = msg.drive.steering_angle
	
	
    def lidarCB(self, msg):
        '''
        This callback is called everytime the laserscanner sends us data.
        This is about 40hz. The message received is a laserscan message
        '''

	#Might want to consider checking what highlevel mux is sending and updating changing looking angle and acceptible distance
	#	this is not implemented yet

	increment = msg.angle_increment
	middle = int((-msg.angle_min)/increment)	 #This is the index in ranges of the laser scan pointing forward
	start = middle - int(math.radians(self.LOOKING_RANGE)/increment) #index of the scan pointing Looking_Range to right
	end = middle + int(math.radians(self.LOOKING_RANGE)/increment) #index of the scan pointing Looking_Range to left
	ranges30 = msg.ranges[start:end] #Only look at ranges in our range
	
	#may want to consider making things that are at more of an angle farther away so if we are 
	#kinds off to an angle of something we dont just stop but has not been added

	#remove any readings that are out of range
	ranges30 = filter(lambda x: x>msg.range_min and x<msg.range_max,ranges30)

	#using max filter
	FILTER_WIDTH = 5
	ranges30 = sc.maximum_filter(ranges30,FILTER_WIDTH)
	ranges30 = ranges30[FILTER_WIDTH/2:-FILTER_WIDTH/2:FILTER_WIDTH/2] #DownSampling
	
	#Find the closest object 
	closest = min(ranges30)

	#If TOO CLOSE STOP
	if closest < self.DANGER_DISTANCE:
	    stop_msg = AckermannDriveStamped()
	    stop_msg.drive.speed = 0.0
            self.pub.publish(stop_msg)

	#IF CLOSEISH SLOW DOWN
	elif closest < self.CAUTION_DISTANCE:
	    stop_msg = AckermannDriveStamped()
	    stop_msg.drive.speed = self.SPEED*(closest-self.DANGER_DISTANCE)/(self.CAUTION_DISTANCE-self.DANGER_DISTANCE)
	    stop_msg.drive.steering_angle = self.ANGLE
            self.pub.publish(stop_msg)
            

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Safety_Node")

    # Init the node
    Safety()

    # Don't let this script exit while ROS is still running
    rospy.spin()

