#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Header, Float32MultiArray
import matplotlib.pyplot as plt

''' When launched with RViz, this class can be used to manually click on points
on the map to print a list of (x,y) coordinates to create a path.'''
class Make_Path():
    def __init__(self):
	self.current_pose = np.zeros(2) #we start at [0,0] unless said
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=1)
        self.clicked_path = []
        self.path = np.array([])

    def clicked_pose(self, msg):
        # if we get a click make a new set of particles
        if type(msg) == PointStamped:
	    x = msg.point.x
            y = msg.point.y
            self.clicked_path.append([x,y])
            rospy.loginfo(self.clicked_path) 

    def visualize(self):
        pass


if __name__=="__main__":
    rospy.init_node("Make_Path")
    pf = Path_Follower()
    rospy.spin()
