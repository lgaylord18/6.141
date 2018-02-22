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

"""
image is 720x1280 (height x width)
(0,0) is top left
(719,1279) is bottom right
"""

class cone_finder:
    def __init__(self):

	self.bridge = CvBridge()
         
    	# publisher for the image we will 
    	self.pub = rospy.Publisher("/controller_info", distance_angle, queue_size=1)
    
    	# subscribe to the rostopic carrying the image we are interested in
    	# "camera/rgb/image_rect_color" is the topic name
    	# Image is the message type
    	# self.processImage is the callback function executed every time we
    	# recieve the message
    	self.sub = rospy.Subscriber("/zed/rgb/image_rect_color",\
	    Image, self.processImage, queue_size=1)
        

	self.coneLower = (5,180,180) #HSV range for cone
	self.coneUpper = (20,256,256) #HSV range for cone


    	self.camXAngle = math.pi/4 #field of view of the camera
    	self.distanceCoef = 225

    	self.width = 640 #image dimension
    	self.height = 360 #image dimension
	self.bumperToCamera = .1524 #distance from camera to bumper


    	# report initalization success
    	rospy.loginfo("Echo Initialized.")

    """
    This is the callback function that is executed every time an Image
    message is received on the rostopic we subscribed to.
    """
    def processImage(self, image_msg):
        # convert rosmsg to cv2 type
        image_cv = self.bridge.imgmsg_to_cv2(image_msg) #get the image in cv2 type

    	hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

    	mask = cv2.inRange(hsv, self.coneLower, self.coneUpper) #check for cone HSV range
    	#mask = cv2.erode(mask, None, iterations=2) #remove noise
    	#mask = cv2.dilate(mask, None, iterations=2) #clean up the image

        
        #find the outline of the blob
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

	if len(cnts) > 0: #only execute if we find a contour
            #find the maximum sized contour
	    c = max(cnts, key=cv2.contourArea)

            #find the min binding box for that larges contour
	    x,y,w,h = cv2.boundingRect(c)

            #calculate distance by the height of the binding box (also subtracet the distance fron the camera tot he bumper)
	    distance = 0.305 * self.distanceCoef/float(h) - self.bumperToCamera

	    #find the center of mass of the largest binding box
            M = cv2.moments(c)
	    center = (int(M["m10"]/M["m00"]),int(M["m01"]/M["m00"])) #(x,y) center of mass of the cone

            #calculate the angle to the cone using its offset from the center of the image.
            left = self.width/2-center[0]
	    theta = math.atan(left*math.tan(self.camXAngle)/(self.width/2)) #angle to center of mass

	    #if w > 10 and h > 10: #outline and draw COM on the contour if close enough 
	        #cv2.rectangle(image_cv,(x,y),(x+w,y+h),(0,255,255),2)
	        #cv2.circle(image_cv,center,5,(0,0,255),-1)

	else:
        #if we did not find a contour, assume the cone the right distance away and infront of us
	    distance = .5
 	    theta = 0

        #cv2.imshow("Frame",mask)
        #cv2.waitKey(1)
	        
        # publish rosmsg 
	distance_angle_msg = distance_angle()
	distance_angle_msg.distance = distance
	distance_angle_msg.angle = theta
        self.pub.publish(distance_angle_msg) 

if __name__=="__main__":
    # initalize the ros node
    rospy.init_node('Cone Finder')

    # create cone_finder to start the image passthrough
    node = cone_finder()

    # continue running cone_finder until node is killed from outside process
    rospy.spin()

