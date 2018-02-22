#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import String, Header, Float32MultiArray
import matplotlib.pyplot as plt
from skimage.morphology import erosion, disk
from scipy.ndimage.morphology import distance_transform_edt as edt
import cv2
import utils as Utils
import heapq

class Path_Planner():
    def __init__(self):
        self.current_pose = np.zeros(3) #we start at [0,0] unless said
        self.target_pose = np.zeros(3)
	self.map_msg = self.get_omap()
        self.pixel_to_meters = self.map_msg.info.resolution
        self.height = self.map_msg.info.height
        self.width = self.map_msg.info.width
        self.map = np.array(self.map_msg.data).reshape((self.height,self.width))
	self.map[self.map==0] = 1
        self.map[self.map!=1] = 0
        self.original_map = self.map
        #self.safe_map = cv2.erode(self.map,None,iterations=1.0/3/self.pixel_to_meters)
	self.map = erosion(self.map,disk(1.0/3/self.pixel_to_meters)) ## erodid the map to make it safe
        self.safe_map = edt(self.map) #at every point tells you the distance to the nearest wall, this is very useful as a heuristic because you want to be as far from walls as possible
        # print self.map[0]
	# print set(self.map[600])
	# print self.map.shape
        # plt.imshow(self.original_map)
        # plt.colorbar()
        # plt.show('hold')

	self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose, queue_size=1)
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=1)
        self.odom_sub  = rospy.Subscriber("/pf/pose/odom", Odometry, self.set_current_pose, queue_size=1)
        self.path_pub = rospy.Publisher("/path",Path,queue_size = 1)


        # print self.location_to_map((0,0))
        # path = [(0,0), (10,0), (10,10), (0,10)]
        # self.visualize(path)

        self.search_alg = self.RRT # self.Astar_circle or self.Astar or self.RRT
        print "here"


    def set_current_pose(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        robot_angle = Utils.quaternion_to_angle(msg.pose.pose.orientation) 
        self.current_pose = np.array([x,y,robot_angle])


    #given a location returns the index position in the map where that location most closely refers to.
    def location_to_map(self,location):
        pos = Utils.world_to_map_slow(location[0],location[1],0, self.map_msg.info)
        return (int(pos[0])%self.width,int(pos[1])%self.height)

    def get_omap(self):
        # this way you could give it a different map server as a parameter
        map_service_name = rospy.get_param("~static_map", "static_map")
        print("getting map from service: ", map_service_name)
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
        return map_msg

    
    def clicked_pose(self, msg):
        # if we get a click make a new set of particles
        if type(msg) == PoseWithCovarianceStamped:
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            theta = Utils.quaternion_to_angle(msg.pose.pose.orientation)
        else:
            x = msg.point.x
            y = msg.point.y
            theta = 0

        self.target_pose = np.array([x,y,theta])
        print self.current_pose
        path = self.search_alg()
        self.publish_path(path)


    def publish_path(self,path):
        path_msg = Path()
        path_msg.header = Utils.make_header("/map", rospy.Time.now())
        path_msg.poses = Utils.particles_to_poses_no_orientation(path)
        self.path_pub.publish(path_msg)


    def Astar_circle(self):
        num_children = 10
        stearing_space = 1.0
        car_len = 0.3
        stearing_angles = np.linspace(-stearing_space,stearing_space,num_children)
        def generate_circle(position):
                return (position,self.safe_map[self.location_to_map(position)[::-1]]*self.pixel_to_meters)

        goal = generate_circle(self.target_pose[0:2])

        def goal_reached(position):
                return np.linalg.norm(goal[0]-position)<(goal[1])

        def heuristic(position):
                return np.linalg.norm(goal[0]-position)

        class Node():
            def __init__(self,pos, radius, index, angle = None, parent=None,cost=0):
                self.pos = np.array(pos)
                self.radius = radius
                self.angle = angle
                self.move = self.radius * 0.5
                self.index = index
                self.parent = parent
                self.cost = cost
                self.heuristic = heuristic(self.pos)*1.5
                self.score = self.cost+self.heuristic

            def getChildren(self):
                children = np.zeros((num_children,3))
                children_angles = np.arctan(np.sin(stearing_angles)*self.move/car_len) + self.angle
                children[:,0:2] = np.array([np.cos(children_angles), np.sin(children_angles)]).T*self.move + self.pos
                children[:,2] = children_angles
                return children

        toVisit = []
        index = self.location_to_map(self.current_pose[0:2])[::-1]
        start_node = Node(self.current_pose[0:2],self.pixel_to_meters*self.safe_map[index],index,self.current_pose[2])
        heapq.heappush(toVisit,(start_node.score,start_node))
        seen_map = np.zeros(self.map.shape).astype(np.uint8)
        visited_map = np.zeros(self.map.shape).astype(np.uint8)
        final_node = None
        print "started"
        while toVisit:
                current_node = heapq.heappop(toVisit)[1]
                if goal_reached(current_node.pos):
                        final_node = current_node
                        break
                if visited_map[current_node.index]:
                        continue
                cv2.circle(visited_map,current_node.index[::-1],int(current_node.move*.2/self.pixel_to_meters),1,-1)
                for position in current_node.getChildren():
                        index = self.location_to_map(position)[::-1]
                        if self.map[index] and not(visited_map[index]) and not(seen_map[index]):
                                new_node = Node(position[0:2],self.pixel_to_meters*self.safe_map[index],index,position[2],current_node,current_node.cost+current_node.move)
                                heapq.heappush(toVisit,(new_node.score,new_node))
                                cv2.circle(seen_map,index[::-1],int(current_node.move*.1/self.pixel_to_meters),1,-1)
        if final_node:
                path = []
                node = final_node
                while node.parent:
                        path.append(node.pos)
                        node = node.parent
                path = path[::-1]
                return path
        else:
                raise Exception('no path to goal with this descritization')


    def RRT(self):
        stearing_space = 1.0
        car_len = 0.2
        num_children = 10
        goal = self.target_pose[0:2]

        def goal_reached(position):
                return np.linalg.norm(goal-position)<(.5) #get to being half a meter from goal

        def heuristic(position):
                return np.linalg.norm(goal-position)#+self.pixel_to_meters*self.safe_map[self.location_to_map(position)[::-1]]

        class Node():
            def __init__(self,pos, radius, index, angle = None, parent=None,cost=0):
                self.pos = np.array(pos)
                self.angle = angle
                self.radius = radius
                self.move = self.radius
                self.index = index
                self.parent = parent
                self.cost = cost
                self.heuristic = heuristic(self.pos)
                self.score = self.cost+self.heuristic*1.5

            def getChildren(self):
                stearing_angles = np.random.uniform(-stearing_space,stearing_space,num_children)
                children = np.zeros((num_children,3))
                children_angles = np.arctan(np.sin(stearing_angles)*self.move/car_len) + self.angle
                children[:,0:2] = np.array([np.cos(children_angles), np.sin(children_angles)]).T*self.move + self.pos
                children[:,2] = children_angles
                return children

        toVisit = []
        index = self.location_to_map(self.current_pose[0:2])[::-1]
        start_node = Node(self.current_pose[0:2],self.pixel_to_meters*self.safe_map[index],index,self.current_pose[2])
        heapq.heappush(toVisit,(start_node.score,start_node))
        seen = np.zeros(self.map.shape)
        final_node = None
        print "started"
        count = 0
        while toVisit:
                current_node = heapq.heappop(toVisit)[1]
                if goal_reached(current_node.pos):
                        final_node = current_node
                        break
                if seen[current_node.index]:
                        continue
                count += 1
                # if count%1000 == 0:
                #         plt.imshow(seen)
                #         plt.show()
                if current_node.parent:
                        cv2.line(seen,self.location_to_map(current_node.pos),self.location_to_map(current_node.parent.pos),1,4)
                for position in current_node.getChildren():
                        index = self.location_to_map(position)[::-1]
                        if self.map[index] and not(seen[index]):
                                new_node = Node(position[0:2],self.pixel_to_meters*self.safe_map[index],index,position[2],current_node,current_node.cost+current_node.move)
                                heapq.heappush(toVisit,(new_node.score,new_node))

        if final_node:
                path = []
                node = final_node
                while node.parent:
                        path.append(node.pos)
                        node = node.parent
                path = path[::-1]
                return path
        else:
                raise Exception('no path to goal with this descritization')


    #Runs Astar from self.current_pose to self.target_pose and returns the path
    def Astar(self, dist_descritization = 0.25): #look at descritization in meters
        mostion_model = np.array([[1,0],[0,1],[-1,0],[0,-1]])*dist_descritization

        goal = self.target_pose[0:2]
        def goal_reached(position):
                return np.linalg.norm(goal-position)<(2*dist_descritization)

        def heuristic(position):
                return np.linalg.norm(goal-position)
                #return np.linalg.norm(goal-position)

        class Node():
                def __init__(self,pos,parent=None,cost=0):
                        self.pos = np.array(pos)
                        self.parent = parent
                        self.cost = cost
                        self.heuristic = heuristic(self.pos)
                        self.score = self.cost+self.heuristic
                def getChildren(self):
                        return self.pos + mostion_model

        toVisit = [Node(self.current_pose[0:2])]
        visited = set()
        final_node = None
        print "started"
        while toVisit:
                toVisit.sort(key=lambda x:x.score)
                current_node = toVisit.pop(0)
                if goal_reached(current_node.pos):
                        final_node = current_node
                        break
                if tuple(current_node.pos) in visited:
                        continue
                visited.add(tuple(current_node.pos))
                for position in current_node.getChildren():
                        if (tuple(position) not in visited) and (self.map[self.location_to_map(position)[::-1]]==1):
                                toVisit.append(Node(position,current_node,current_node.cost+dist_descritization))
        if final_node:
                path = []
                node = final_node
                while node.parent:
                        path.append(node.pos)
                        node = node.parent
                path = path[::-1]
                return path
        else:
                raise Exception('no path to goal with this descritization')        

        
    #Visualizes the path over the map
    def visualize(self, pathlist):
        #pathlist is a temporarily-named parameter to represent a list of (x,y) coordinates (tuples) to represent the path
        #image is a temporarily-named paramter to represent the map, np array of arrays (2D array)
        
        #make a copy of the image which is a numpy array
        cvimage = np.copy(self.map).astype(np.uint8)
        
        #draw the path on the image
        for i in range(len(pathlist)-1):
                startPoint = self.location_to_map((pathlist[i][0], pathlist[i][1]))
                endPoint = self.location_to_map((pathlist[i+1][0], pathlist[i+1][1]))
                cv2.line(cvimage, startPoint, endPoint, 2, 10) #draw a line representing one segment of the path 
        
        #display the image
        plt.imshow(cvimage)
        plt.show('hold')

# this function can be used to generate flame graphs easily
# def make_flamegraph(filterx=None):
#     import flamegraph, os
#     perf_log_path = os.path.join(os.path.dirname(__file__), "../tmp/perf.log")
#     flamegraph.start_profile_thread(fd=open(perf_log_path, "w"),
#                                     filter=filterx,
#                                     interval=0.001)

if __name__=="__main__":
    rospy.init_node("Path_Planner")
    pf = Path_Planner()
    #make_flamegraph()
    rospy.spin()
