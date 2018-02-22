#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt

#This class simply publishes the lidar data values for directly in front of the robot
#We used it to assess how accurate the lidar data is in order to create an accurate distribution for out particle filter
class LidarData():
    def __init__(self):
    	#init subscriber
	self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.lidarCB, queue_size=1)
        self.lidar_data = []  #init list to hold lidar data over time

    def lidarCB(self, msg):
	
	while len(self.lidar_data) < 1001:
	    #get the dist value for lidar data that is directly in front of the robot and add to list
	    length = len(msg.ranges)/2
	    dist = msg.ranges[length]
	    self.lidar_data.append(dist)
	    #print lidar data
            rospy.loginfo(str(dist) + ' ' + str(len(self.lidar_data)))

	    #create plot of lidar_data
	    if len(self.lidar_data) == 1000:
	        plt.hist(self.lidar_data)
	        plt.title('Laser Scan Data')
	        plt.xlabel('Distance')
	        plt.ylabel('Frequency')
                plt.show
	    

#spins the ROS node to continuously input data from LIDAR
if __name__=="__main__":
    rospy.init_node("LidarData_Node")

    LidarData()
    rospy.spin()
