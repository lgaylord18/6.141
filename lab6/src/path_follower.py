#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import math
import tf
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Header, Float32MultiArray
import utils as Utils

class Path_Follower():
    def __init__(self):
        self.path = None
	    # self.pose = np.zeros(0) #we start at [0,0] unless said
        # self.path_pub = rospy.Subscriber("/path", Path, self.follow_path, queue_size = 1)
        self.odom_sub  = rospy.Subscriber("/pf/pose/odom", Odometry, self.follow_path, queue_size=1)
        self.path_sub  = rospy.Subscriber("/path", Path, self.set_path, queue_size=1)
        self.pub = rospy.Publisher("/vesc/high_level/ackermann_cmd_mux/input/nav_0",\
            AckermannDriveStamped, queue_size =1 )

        # self.path = np.array([])
        self.look_ahead = 2.0
        

    def pose_to_particle(self,pose):
        x = pose.pose.position.x
        y = pose.pose.position.y
        return np.array([x,y])

    def set_path(self, msg):
        self.path = map(self.pose_to_particle,msg.poses)

    '''
    For a given path, returns the index of the point closest to the pose
    '''
    def find_nearest_point(self, pose, path):
        pose = np.array(pose)
        min_index = 0
        min_dist = float('inf')
        for i in xrange(len(path)):
                dist = np.linalg.norm(np.array(path[i])-pose)
                if dist < min_dist:
                        min_dist = dist
                        min_index = i
        return min_index


    '''
    Finds if a line segment of two points [[x1,y1], [x2,y2]] intersect with the circle with
    the center [x,y] and radius.
    Returns the intersection point, else False.

    This method is based on the following:
    http://codereview.stackexchange.com
    /questions/86421/line-segment-to-circle-collision-algorithm
    '''
    def find_intersection(self, center, radius, segment):
        # d1 = np.linalg.norm(center-np.array(segment[0]))
        # d2 = np.linalg.norm(center-np.array(segment[1]))
        # if d2 >= radius and d1< radius:
        #         return True, d2
        # else:
        #         return False, None
        r = radius
        C = center
        segment = np.array(segment)
        P_1 = segment[0]
        P_2 = segment[1]
        V = P_2 - P_1

        a = np.dot(V, V)
        b = 2 * np.dot(V, P_1 - C)
        c = np.dot(P_1, P_1) + np.dot(C, C) - 2 * np.dot(P_1, C) - r**2

        delta = b**2 - 4*a*c
        if delta < 0:
            return False, None

        sqrt_delta = math.sqrt(delta)
        t_1 = (-b + sqrt_delta) / (2 * a)
        t_2 = (-b - sqrt_delta) / (2 * a)
        S_1 = P_1 + t_1 * V
        S_2 = P_1 + t_2 * V

        intersection = None
        intersects = True

        if 0 <= t_1 <= 1 and 0 <= t_2 <= 1:
            dist_1 = np.linalg.norm(P_2 - S_1)
            dist_2 = np.linalg.norm(P_2 - S_2)
            intersection = S_2 if dist_1 > dist_2 else S_1
        elif 0 <= t_1 <= 1:
            intersection = S_1
        elif 0 <= t_2 <= 1:
            intersection = S_2
        else:
            intersects = False
        
        return intersects, intersection

    '''
    Finds the pursuit_target with the following algorithm:
        Go through the line segments of the path, until 
        a segment is found that intersects with
        the circle of radius lookh_ahead and center with the position of robot

    This is based on the Pure Pursit Hints by Corey Walsh:
    https://piazza.com/class/iz4pm04fb5f2yz?cid=188
    '''
    def find_pursuit_target(self, pose, look_ahead, path):
        for i in range(len(path)-1):
            segment = [path[i], path[i+1]]
            intersects, intersection = self.find_intersection(pose, look_ahead, segment)
            if intersects:
                return intersection
	    else:
		if np.linalg.norm(np.array(pose)-np.array(path[i]))>look_ahead:
		    break
        return path[0]


    def follow_path(self, msg):
        if self.path:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            robot_angle = Utils.quaternion_to_angle(msg.pose.pose.orientation) 
            # find the pursuit point and angle to it
            robot_position = [x, y]
            index = self.find_nearest_point(robot_position, self.path)
            path_left = self.path[index:]
            target_point = self.find_pursuit_target([x,y], self.look_ahead, path_left)

            if np.linalg.norm(target_point-self.path[-1])<0.25:
                print "goal reached"
                while np.linalg.norm(target_point-self.path[-1])>0.1 and np.linalg.norm(target_point-self.path[-1])<0.25:
                        #put together message and publish to high level mux
                        new_msg = AckermannDriveStamped()
                        new_msg.drive.steering_angle = 0.0
                        new_msg.drive.speed = 0.5
                        self.pub.publish(new_msg)
                path = None
                return
        
            direction_vector = np.array(target_point) - np.array(robot_position)
            print direction_vector
            direction_angle = np.arctan(direction_vector[1]/direction_vector[0])
            if direction_vector[1] < 0 and direction_vector[0] < 0:
                direction_angle = -1*(math.pi-direction_angle)
            if direction_vector[1] > 0 and direction_vector[0] < 0:
                direction_angle += math.pi
            print direction_angle
            pursuit_angle = direction_angle - robot_angle
            print pursuit_angle

            ### pure pursuit
            angle = pursuit_angle
            car_length = 0.46    #meters
            L_1 = self.look_ahead #our distance to the target point

            #set steering angle based on pure pursuit method covered in lecture
            steering_angle = math.atan(2 * car_length * math.sin(angle) / L_1)   
        
            #put together message and publish to high level mux
            new_msg = AckermannDriveStamped()
            new_msg.drive.steering_angle = steering_angle
            new_msg.drive.speed = 5.0
            self.pub.publish(new_msg)

if __name__=="__main__":
    rospy.init_node("Path_Follower")
    pf = Path_Follower()
    rospy.spin()
