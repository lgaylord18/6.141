#!/usr/bin/env python
import rospy
import numpy as np
import math
import scipy.signal as ss
from sklearn import linear_model
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Header
from ackermann_msgs.msg import AckermannDriveStamped
from lab3.msg import distance_angle

import numpy as np

class WallScanner():
    def __init__(self):
        
        # Init subscribers and publishers
        self.sub = rospy.Subscriber("/scan", LaserScan,\
                self.lidarCB, queue_size=1)
                
        self.pub = rospy.Publisher("/controller_info",\
                distance_angle, queue_size =1 )
                
        rospy.loginfo("Safety node initialized")
        
    def lidarCB(self, msg):
	#call generate_line to get distance and angle from wall
        distance_to_wall,angle_to_wall = self.generate_line(msg)
	
	#publish the data
	PositionMsg = distance_angle()
	PositionMsg.distance = distance_to_wall
	PositionMsg.angle = angle_to_wall
        self.pub.publish(PositionMsg)
        
    
    def generate_line(self, msg):
	#find out how many measurements there are from angle_min to -30 deg right of the robot
	numMeasurements=int((math.radians(-30)-msg.angle_min)/msg.angle_increment)
	ranges = msg.ranges[0:numMeasurements] #get the measurements to our right (-30deg to angle_min)

	#add angles to ranges so its a list of (distance,angle)
	angles = np.arange(msg.angle_min,msg.angle_min+numMeasurements*msg.angle_increment,msg.angle_increment)
	ranges_angle = np.vstack([np.array(ranges),angles]).T

	#filter out distances out of range
	ranges_angle = filter(lambda x: x[0]>msg.range_min and x[0] < msg.range_max,ranges_angle)

	#median filter of bucket size on both distances and angle
        bucket_size = 5
	smoothed_distangle_data = ss.medfilt(ranges_angle,[bucket_size,1])[bucket_size:-bucket_size:bucket_size]


	#now from the filtered (distance,angles) find the xs and ys
	xs = map(lambda x: -math.sin(x[1])*x[0],smoothed_distangle_data)
	ys = map(lambda x:  math.cos(x[1])*x[0],smoothed_distangle_data)

	#make them np arrays and of the right shape ((n,1) normally (n,))
	x = np.array(xs).reshape(-1,1)
	y = np.array(ys).reshape(-1,1)

	#setup and run ransac
	model_ransac = linear_model.RANSACRegressor(linear_model.LinearRegression())
	model_ransac.fit(x,y)

	#get the slope(m) and intercept(c) from ransac
	m = model_ransac.estimator_.coef_[0][0]
	c = model_ransac.estimator_.intercept_[0]
	
	#calculate distance and angle to wall using m and c then return it
	normal = np.array([-m,1])
	distance_to_wall = abs(c/np.linalg.norm(normal))
	angle_to_wall = np.sign(m)*math.pi/2-math.asin(m/np.linalg.norm(normal))
	return(distance_to_wall,angle_to_wall)
        

if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Wall_Scanner_Node")

    # Init the node
    WallScanner()

    # Don't let this script exit while ROS is still running
    rospy.spin()
