#!/usr/bin/env python
import numpy as np
import argparse
import scipy.signal
import copy
import scipy.ndimage as sc
import cv2  # the open cv library packge
import rospy # standard ros with python package
from challenge0.msg import cone
from sensor_msgs.msg import Image  # the rostopic message we subscribe/publish 
from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
import utils as Utils
import time
import math

"""
image is 720x1280 (height x width)
(0,0) is top left
(719,1279) is bottom right
"""

"""
Edge cases regarding cones
Multiple cones in the scene: Finds the cone with the largest contour, i.e. closer to go
"""


class cone_finder:
    def __init__(self):
    	self.pose = np.zeros(3)
    	self.objects = []


    	# Ranges for orange cone
	self.coneOrangeLower = (7,80,80) #HSV range for orange cone
	self.coneOrangeUpper = (14,230,230) #HSV range for orange cone
        self.coneGreenLower = (55,50,30) #HSV range for green cone
        self.coneGreenUpper = (88,230,230) #HSV range for green cone

	self.LOOKING_RANGE = math.pi/3.0
	self.increment = None
	self.start = None
	self.middle = None
	self.end = None


    	self.camXAngle = math.pi/4 #field of view of the camera

	self.width = 1280 #image dimension
    	self.height = 720 #image dimension

	self.map_msg = self.get_omap()
    	array_255 = np.array(self.map_msg.data).reshape((self.map_msg.info.height, self.map_msg.info.width))
    	self.permissible_region = np.zeros((self.map_msg.info.height, self.map_msg.info.width), dtype=float)
        self.permissible_region[array_255==0] = 1
        self.permissible_region = cv2.erode(self.permissible_region, None, iterations=13)
	self.O_permissible_region = copy.deepcopy(self.permissible_region)
        #cv2.imshow('frame', self.permissible_region)
        #cv2.waitKey(1) # just want to see what it looks like

        self.odom_sub  = rospy.Subscriber(rospy.get_param("~odom_topic"), Odometry, self.odomCB, queue_size=1)
    	self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.lidarCB, queue_size=1)
        self.im_sub_l = rospy.Subscriber("/zed/left/image_rect_color", Image, self.processImageL, queue_size=1)
        self.im_sub_r = rospy.Subscriber("/zed/right/image_rect_color", Image, self.processImageR, queue_size=1)

	self.click_sub = rospy.Subscriber("/clicked_point",PointStamped,self.reset,queue_size=1)

	self.bridge = CvBridge()
        
    	# publisher for the image we will 
    	self.pub = rospy.Publisher("/cone_info", cone, queue_size=1)
	self.cone_viz_pub = rospy.Publisher("/cone_point", Marker, queue_size = 1)

    	# report initalization success
    	rospy.loginfo("Echo Initialized.")

	self.constant = (self.width/2)/math.tan(self.camXAngle)
	self.image_hsv_L = np.zeros((self.height,self.width,3))
	self.image_hsv_R = np.zeros((self.height,self.width,3))
	self.main()



    def reset(self,msg):
	print "wiping"
	self.permissible_region = copy.deepcopy(self.O_permissible_region)
	cone_msg = cone()
	cone_msg.distance = 0
	cone_msg.angle = 0
	cone_msg.exists = True
	cone_msg.color = "no"
	self.pub.publish(cone_msg)


    def main(self):
	i = 0
	while True:
	    for location in self.objects:
	    	angle = location[1]
	    	index = max([min([int(self.width-(math.tan(angle)*self.constant + self.width/2)),self.width]),0])
	    	if index <= 200:
	    	    left = 0
	    	    right = 400

	    	elif index >= self.width-400:
	    	    left = self.width-401
	    	    right = self.width-1

	    	else:
	    	    left = index - 200
	    	    right = index + 200
	    	if index < self.width/2:
	    	    #cv2.imshow("FRAME",self.image_hsv_L[int(self.height*0.3):int(self.height*0.9),left:right])
		    #cv2.waitKey(1)
		    hsv = self.image_hsv_L[int(self.height*0.45):int(self.height*0.9),left:right]
		else:
	    	    #cv2.imshow("FRAME",self.image_hsv_R[int(self.height*0.3):int(self.height*0.9),left:right])
		    #cv2.waitKey(1)
		    hsv = self.image_hsv_R[int(self.height*0.45):int(self.height*0.9),left:right]
		maskOrange = cv2.inRange(hsv, self.coneOrangeLower, self.coneOrangeUpper) #check for cone HSV range
    		maskOrange = cv2.erode(maskOrange, None, iterations=1) #remove noise
        	maskGreen = cv2.inRange(hsv, self.coneGreenLower, self.coneGreenUpper) #check for cone HSV range
		maskGreen = cv2.erode(maskGreen, None, iterations=1) #remove noise
		
		#cv2.imshow("mask",maskOrange)
		#cv2.waitKey(1)

		#cv2.imshow("mask",maskGreen)
		#cv2.waitKey(1)

		cntsOrange = cv2.findContours(maskOrange.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        	cntsGreen = cv2.findContours(maskGreen.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		area_O = 0
		if len(cntsOrange)>0:
		    c = max(cntsOrange,key=cv2.contourArea)
		    area_O = cv2.contourArea(c)
		area_G = 0
		if len(cntsGreen)>0:
		    c = max(cntsGreen,key=cv2.contourArea)
		    area_G = cv2.contourArea(c)
		if area_O > area_G:
		    color = "orange"
		else:
		    color = "green"
		if max([area_O,area_G])>400:
		    map_location = self.relative_to_map(location[0],location[1])
		    self.permissible_region = copy.deepcopy(self.O_permissible_region)
		    cv2.circle(self.permissible_region,map_location,20,0,-1)
		    #cv2.imshow('frame', self.permissible_region)
        	    #cv2.waitKey(1) # just want to see what it looks like
		    #print location
		    #print color
		    world = self.relative_to_world(location[0],location[1])
		    cone_msg = cone()
		    cone_msg.distance = location[0]
		    cone_msg.angle = location[1]
		    cone_msg.exists = False
		    cone_msg.color = color
		    self.pub.publish(cone_msg)
		    if True: #if visualize
			if color == "orange":
			    c = [1.0,0.0,0.0]
			else:
			    c = [0.0,1.0,0.0]
			self.cone_viz_pub.publish(Utils.make_circle_marker(world, 0.5, c, "/map", "cone_viz", 0, 60))
		    break
		

	    	


   #given a location returns the index position in the map where that location most closely refers to.
    def location_to_map(self,location):
        pos = Utils.world_to_map_slow(location[0],location[1],0, self.map_msg.info)
        return (int(pos[0])%self.map_msg.info.width,int(pos[1])%self.map_msg.info.height)

    def relative_to_world(self,distance,angle):
    	world_angle = self.pose[2]+angle
    	return np.array([self.pose[0]+np.cos(world_angle)*distance,self.pose[1]+np.sin(world_angle)*distance])

    def relative_to_map(self,distance,angle):
    	return self.location_to_map(self.relative_to_world(distance,angle))

    def safe_spot(self,position):
    	return self.permissible_region[self.location_to_map(self.relative_to_world(position[0],position[1]))[::-1]]==1
		
    def get_omap(self):
	# this way you could give it a different map server as a parameter
	map_service_name = rospy.get_param("~static_map", "static_map")
	print("getting map from service: ", map_service_name)
	rospy.wait_for_service(map_service_name)
	map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
	return map_msg

    def odomCB(self, msg):
	''' Extracts robot state information from the message, and executes pure pursuit control.
	'''
	self.pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, Utils.quaternion_to_angle(msg.pose.pose.orientation)])


    def lidarCB(self,msg):
    	if self.increment == None:
    		self.increment = msg.angle_increment
		self.middle = int((-msg.angle_min)/self.increment)	 #This is the index in ranges of the laser scan pointing forward
		self.start = self.middle - int(self.LOOKING_RANGE/self.increment) #index of the scan pointing Looking_Range to right
		self.end = self.middle + int(self.LOOKING_RANGE/self.increment) #index of the scan pointing Looking_Range to left
		self.min_range = msg.range_min
		self.max_range = msg.range_max
		self.FILTER_WIDTH = int(math.pi/60/self.increment)
		self.angles = np.linspace(-self.LOOKING_RANGE,self.LOOKING_RANGE,(2*self.LOOKING_RANGE)/self.increment/(self.FILTER_WIDTH))
	ranges = msg.ranges #sc.maximum_filter(msg.ranges,3)
	clean_msg = np.min(np.array(ranges[self.start:self.end]).reshape((-1,self.FILTER_WIDTH)),axis=1)
    	scan = zip(clean_msg,self.angles)
    	scan = filter(lambda x:x[0]<2 and x[0]>0.1,scan)
    	scan.sort(key=lambda x:x[0])
    	self.objects = filter(self.safe_spot,scan)

    """
    This is the callback function that is executed every time an Image
    message is received on the rostopic we subscribed to.
    """
    def processImageR(self, image_msg):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg) #get the image in cv2 type
        self.image_hsv_R = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

    """
    This is the callback function that is executed every time an Image
    message is received on the rostopic we subscribed to.
    """
    def processImageL(self, image_msg):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg) #get the image in cv2 type
        self.image_hsv_L = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)



if __name__=="__main__":
    # initalize the ros node
    rospy.init_node('Cone Finder')

    # create cone_finder to start the image passthrough
    node = cone_finder()

    # continue running cone_finder until node is killed from outside process
    rospy.spin()

