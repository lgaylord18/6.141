#!/usr/bin/env python
import numpy as np
import argparse
import cv2  # the open cv library packge
import rospy # standard ros with python package
from lab4.msg import distance_angle
from sensor_msgs.msg import Image  # the rostopic message we subscribe/publish 
from cv_bridge import CvBridge
import time
import math


#cv2.circle(circle,(width/2,height),width/2,1,thickness=100)

"""
Image is 360x640 (height x width)
(0,0) is top left
(719,1279) is bottom right
"""

class line_finder:
    def __init__(self):

	self.bridge = CvBridge()
         
    	# publisher for the image we will 
    	self.pub_low = rospy.Publisher("/controller_info_low", distance_angle, queue_size=1)
        self.pub_high = rospy.Publisher("/controller_info_high", distance_angle, queue_size=1)
    
    	# self.processImage is the callback function executed every time we
    	# recieve the message
    	self.sub_left = rospy.Subscriber("/zed/left/image_rect_color",\
	    Image, self.processLeftImage, queue_size=1)
        self.sub_right = rospy.Subscriber("/zed/right/image_rect_color",\
            Image, self.processRightImage, queue_size=1)

	self.Lower = (5,100,100) #HSV range for line_finder
	self.Upper = (20,256,256) #HSV range for line_finder
	self.lowBandRange = (3.0/4,1.0/1)
        self.highBandRange = (1.0/2,5.0/8)

    	self.camXAngle = math.pi/4 #field of view of the camera
        self.camHeight = 0.16 #height from the camera
    	self.distanceCoef = 225

    	self.width = 672 #image dimension
    	self.height = 376 #image dimension
	self.bumperToCamera = .1524 #distance from camera to bumper
        self.leftImageMsg = None
        self.rightImageMsg = None

    	# report initalization success
    	rospy.loginfo("Echo Initialized.")

    #if pixels within range, set to 1, if not within range set to 0
    def mask(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.Lower, self.Upper)
        mask = cv2.erode(mask, None, iterations=0)
        mask = cv2.dilate(mask, None, iterations=4)
        return mask

    #determine center of mass of contour
    def findCM(self,mask):
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            #x,y,w,h = cv2.boundingRect(c)
            M = cv2.moments(c)
            center = (int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"]))
            return center
        return None

    """
    This is the callback function that is executed every time an Image
    message is received on the rostopic /zed/left/image_rect_color we subscribed to.
    It updates the global leftImageMsg variable.
    """
    def processLeftImage(self, image_msg):
        self.leftImageMsg = image_msg
    
    """
    This is the callback function that is executed every time an Image
    message is received on the rostopic /zed/right/image_rect_color we subscribed to.
    Grabs the current rightImageMsg and leftImageMsg variables and
    publishes the average_distance_angle_msg
    """ 
    def processRightImage(self, image_msg):
        self.rightImageMsg = image_msg

        if self.leftImageMsg == None: #wait till left has posted an image
            return

        # Process the left and right images
        left_low_distance_angle_msg = self.processImage(self.leftImageMsg,self.lowBandRange)
        left_high_distance_angle_msg = self.processImage(self.leftImageMsg,self.highBandRange)

        right_low_distance_angle_msg = self.processImage(self.rightImageMsg,self.lowBandRange)
        right_high_distance_angle_msg = self.processImage(self.rightImageMsg,self.highBandRange)

        # Average the low image distance and angle for an accurate symmetric view
        average_low_distance_angle_msg = distance_angle()
        average_low_distance_angle_msg.distance = (left_low_distance_angle_msg.distance + \
         right_low_distance_angle_msg.distance)/2.0
        average_low_distance_angle_msg.angle = (left_low_distance_angle_msg.angle + \
         right_low_distance_angle_msg.angle)/2.0

        # averaging for high image
        average_high_distance_angle_msg = distance_angle()
        average_high_distance_angle_msg.distance = (left_high_distance_angle_msg.distance + \
         right_high_distance_angle_msg.distance)/2.0
        average_high_distance_angle_msg.angle = (left_high_distance_angle_msg.angle + \
         right_high_distance_angle_msg.angle)/2.0

        #publish the high and low data
        #rospy.loginfo(average_distance_angle_msg.angle)     
        self.pub_low.publish(average_low_distance_angle_msg) 
        self.pub_high.publish(average_high_distance_angle_msg)    

    """
    Processes the image message and outputs a distance_angle() message
    """
    def processImage(self, image_msg,include_range):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg) #get the image in cv2 type
	image = image_cv[int(include_range[0]*len(image_cv)):int(include_range[1]*len(image_cv))] #get the lower pixels
        mask = self.mask(image) #get the mask for the selected pixels
        center = self.findCM(mask) #find the center of mass of the largest contour

        #calculates robot's angle (theta) to the center of mass of the largest contour
        if center:
	    left = self.width/2-center[0]
            y = self.height-(center[1]+len(image_cv)*include_range[0])
            
            theta = math.atan(left*math.tan(self.camXAngle)/(self.width/2))
	    distance = .192*math.exp(.01*y)*(1.0/math.cos(theta))
	
        else:
            theta = 0
            distance = 0.5

        #rospy.loginfo(theta)
        #return the angle to the center of mass
    	distance_angle_msg = distance_angle()
	distance_angle_msg.distance = distance
	distance_angle_msg.angle = theta
        return distance_angle_msg




if __name__=="__main__":
    # initalize the ros node
    rospy.init_node('Line Finder')

    # create line_finder to start the image passthrough
    node = line_finder()

    # continue running line_finder until node is killed from outside process
    rospy.spin()


