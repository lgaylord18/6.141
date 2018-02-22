#!/usr/bin/env python

'''
Lab 5 Starter Code

- Outlines a basic implementation of particle filter localization algorithm
- Initializes RangeLibc
- Uses ROS params for easier configuration
- Only sends visualization if someone is listening
- Uses locks to avoid concurreny errors
- Includes code for visualizing discretized sensor models with matplotlib
- Includes helper functions
    - Timer
    - CircularArray
    - Utils
        - coordinate space conversions
        - useful ROS object construction functions

While developing, be careful of:
    - coordinate conversions
    - vectorization with numpy
    - caching and reusing large buffers
    - concurrency problems
    - implement visualization early to help debugging

To launch:

    first start the map server: 
    $ roslaunch lab5 map_server.launch
    then launch this code with:
    $ roslaunch lab5 localize.launch

Written by Corey Walsh for Spring 2017 6.141 Lab 5

'''

import rospy
import numpy as np

from std_msgs.msg import String, Header, Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from scipy.stats import norm
import tf.transformations
import tf
import matplotlib.pyplot as plt
import math
import random
import range_libc
import time

from threading import Lock

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

class CircularArray(object):
    """ A simple circular array implementation 

        You can append any number of elements, but only the last N will be kept
        where N is the integer passed to the constructor. Useful for smoothing values.
    """
    def __init__(self, size):
        self.arr = np.zeros(size)
        self.ind = 0
        self.num_els = 0

    def append(self, value):
        if self.num_els < self.arr.shape[0]:
            self.num_els += 1
        self.arr[self.ind] = value
        self.ind = (self.ind + 1) % self.arr.shape[0]

    # returns the mean of maintained elements
    def mean(self):
        return np.mean(self.arr[:self.num_els])

    # returns the median of maintained elements
    def median(self):
        return np.median(self.arr[:self.num_els])

class Timer:
    ''' A simple timer class to track iterations per second

        Uses a CircularArray to smooth FPS values.
        Pass an integer to indicate how many values to maintain.
    '''
    def __init__(self, smoothing):
        self.arr = CircularArray(smoothing)
        self.last_time = time.time()

    ''' Call this on every iteration
    '''
    def tick(self):
        t = time.time()
        self.arr.append(1.0 / (t - self.last_time))
        self.last_time = t

    ''' Call this to check recent average calls per second 
    '''
    def fps(self):
        return self.arr.mean()

class Utils(object):
    """ Helper functions """
        
    @staticmethod
    def angle_to_quaternion(angle):
        """Convert an angle in radians into a quaternion _message_."""
        return Quaternion(*tf.transformations.quaternion_from_euler(0, 0, angle))

    @staticmethod
    def quaternion_to_angle(q):
        """Convert a quaternion _message_ into an angle in radians.
        The angle represents the yaw.
        This is not just the z component of the quaternion."""
        x, y, z, w = q.x, q.y, q.z, q.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((x, y, z, w))
        return yaw

    @staticmethod
    # gives a rotation matrix for rotating coordinates by theta
    # not recommended for direct use since this will be slow
    # instead apply the same math over a whole array all at once when possible
    def rotation_matrix(theta):
        cosine, sine = np.cos(theta), np.sin(theta)
        return np.matrix([[cosine, -sine], [sine, cosine]])

    @staticmethod
    def particle_to_pose(particle):
        pose = Pose()
        pose.position.x = particle[0]
        pose.position.y = particle[1]
        pose.orientation = Utils.angle_to_quaternion(particle[2])
        return pose

    @staticmethod
    def particles_to_poses(particles):
        return map(Utils.particle_to_pose, particles)

    @staticmethod
    def make_header(frame_id, stamp=None):
        if stamp == None:
            stamp = rospy.Time.now()
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header

    @staticmethod
    def point(npt):
        pt = Point32()
        pt.x = npt[0]
        pt.y = npt[1]
        return pt

    @staticmethod
    def points(arr):
        return map(Utils.point, arr)


    # the following functions are for converting to/from map and world coordinates
    # useful for converting from world poses to array indices in the "self.permissible" array
    # they may not be exactly what you need, and you may neet to switch your
    # x and y coorindates before indexing into the permissible array

    # converts map space coordinates to world space coordinates
    # this version is slow but easy to follow logically
    @staticmethod
    def map_to_world_slow(x,y,t,map_info):
        scale = map_info.resolution
        angle = Utils.quaternion_to_angle(map_info.origin.orientation)
        rot = Utils.rotation_matrix(angle)
        trans = np.array([[map_info.origin.position.x],
                          [map_info.origin.position.y]])

        map_c = np.array([[x],
                          [y]])
        world = (rot*map_c) * scale + trans

        return world[0,0],world[1,0],t+angle

    # converts world space coordinates to map space coordinates
    @staticmethod
    def world_to_map_slow(x,y,t, map_info):
        scale = map_info.resolution
        angle = Utils.quaternion_to_angle(map_info.origin.orientation)
        rot = Utils.rotation_matrix(-angle)
        trans = np.array([[map_info.origin.position.x],
                          [map_info.origin.position.y]])

        world = np.array([[x],
                          [y]])
        map_c = rot*((world - trans) / float(scale))
        return map_c[0,0],map_c[1,0],t-angle

    @staticmethod
    # same as above but faster, operates on an Nx3 array of poses
    def map_to_world(poses,map_info):
        scale = map_info.resolution
        angle = Utils.quaternion_to_angle(map_info.origin.orientation)

        # rotation
        c, s = np.cos(angle), np.sin(angle)
        # we need to store the x coordinates since they will be overwritten
        temp = np.copy(poses[:,0])
        poses[:,0] = c*poses[:,0] - s*poses[:,1]
        poses[:,1] = s*temp       + c*poses[:,1]

        # scale
        poses[:,:2] *= float(scale)

        # translate
        poses[:,0] += map_info.origin.position.x
        poses[:,1] += map_info.origin.position.y
        poses[:,2] += angle
        
    @staticmethod
    # same as above but faster, operates on an Nx3 array of poses
    def world_to_map(poses, map_info):
        # operates in place
        scale = map_info.resolution
        angle = -Utils.quaternion_to_angle(map_info.origin.orientation)

        # translation
        poses[:,0] -= map_info.origin.position.x
        poses[:,1] -= map_info.origin.position.y

        # scale
        poses[:,:2] *= (1.0/float(scale))

        # rotation
        c, s = np.cos(angle), np.sin(angle)
        # we need to store the x coordinates since they will be overwritten
        temp = np.copy(poses[:,0])
        poses[:,0] = c*poses[:,0] - s*poses[:,1]
        poses[:,1] = s*temp       + c*poses[:,1]
        poses[:,2] += angle

class ParticleFiler():
    def __init__(self):

        #initializing variables we need in the future
        self.oldOdomX = 0
        self.oldOdomY = 0
        self.oldOdomTheta = 0
        self.action = np.zeros(3);

        self.laser_angles = None
        self.first_sensor_update = True
	self.lidar_initialized = False
	self.inferred_pose = np.zeros(3);

        # parameters
        self.MAX_PARTICLES = int(rospy.get_param("~max_particles"))
        self.MAX_VIZ_PARTICLES = int(rospy.get_param("~max_viz_particles"))
        self.MAX_RANGE_METERS = float(rospy.get_param("~max_range"))
        self.MAX_RANGE_PX = None
        # cddt and glt range methods are discrete, this defines number of discrete thetas
        self.THETA_DISCRETIZATION = int(rospy.get_param("~theta_discretization"))
        self.WHICH_RANGE_METHOD = rospy.get_param("~range_method", "cddt")

        # various data containers used in the MCL algorithm
        self.map_info = None
        self.map_initialized = False
        self.range_method = None

        # use this lock for controlling accesses to the particles
        # necessary for avoiding concurrency errors
        self.state_lock = Lock()

        # when possible, use these variables to cache large arrays and only make them once
        self.queries = None
        self.ranges = None
        self.sensor_model_table = None

        # particle poses and weights - particles should be N by 3
        self.particles = np.zeros((self.MAX_PARTICLES, 3),dtype=np.float32)
	self.particles += np.random.normal(0.0, 0.2, (self.MAX_PARTICLES, 3))

        # uniform prior
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)

        # initialize the state
        self.get_omap()
        self.precompute_sensor_model()

        # these topics are for visualization, feel free to add, remove, or change as you see fit
        self.pose_pub      = rospy.Publisher("/pf/viz/inferred_pose", PoseStamped, queue_size = 1)
        self.particle_pub  = rospy.Publisher("/pf/viz/particles", PoseArray, queue_size = 1)
        self.pub_fake_scan = rospy.Publisher("/pf/viz/fake_scan", LaserScan, queue_size = 1)
        
        # use this for your inferred transformations
        self.pub_tf = tf.TransformBroadcaster()

	#rospy.get_param("~odometry_topic", "/odom")
        self.odom_sub  = rospy.Subscriber(rospy.get_param("~odometry_topic", "/odom"), Odometry, self.odomCB, queue_size=1)

        # these topics are to receive data from the racecar
        self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.lidarCB, queue_size=1)

	
        # these integrate with RViz
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose, queue_size=1)
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=1)

        print "Finished initializing, waiting on messages..."

    def get_omap(self):
        # this way you could give it a different map server as a parameter
        map_service_name = rospy.get_param("~static_map", "static_map")
        print("getting map from service: ", map_service_name)
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
		
		
        self.map_info = map_msg.info
        oMap = range_libc.PyOMap(map_msg)
        # this value is the max range used internally in RangeLibc
        # it also should be the size of your sensor model table
        self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS / self.map_info.resolution)

        # initialize range method
        print "Initializing range method:", self.WHICH_RANGE_METHOD
        if self.WHICH_RANGE_METHOD == "bl":
            self.range_method = range_libc.PyBresenhamsLine(oMap, self.MAX_RANGE_PX)
        elif "cddt" in self.WHICH_RANGE_METHOD:
            self.range_method = range_libc.PyCDDTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
            if self.WHICH_RANGE_METHOD == "pcddt":
                print "Pruning..."
                self.range_method.prune()
        elif self.WHICH_RANGE_METHOD == "rm":
            self.range_method = range_libc.PyRayMarching(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RANGE_METHOD == "rmgpu":
            self.range_method = range_libc.PyRayMarchingGPU(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RANGE_METHOD == "glt":
            self.range_method = range_libc.PyGiantLUTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
        print "Done loading map"

         # 0: permissible, -1: unmapped, large value: blocked
        array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        # 0: not permissible, 1: permissible
        # this may be useful for global particle initialization - don't initialize particles in non-permissible states
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255==0] = 1
        self.map_initialized = True

    def publish_tf(self, stamp=None):
        """ Publish a tf from map to base_link. """
        if stamp == None:
            stamp = rospy.Time.now()
        # http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20(Python)
        # get a quaternion of our current angle then publish our inferred_pose as the tf between base_link and the map
	q = Utils.angle_to_quaternion(self.inferred_pose[2])
        self.pub_tf.sendTransform((self.inferred_pose[0], self.inferred_pose[1], 0.0),(q.x,q.y,q.z,q.w),stamp,"base_link","map")

    def visualize(self):
        if self.pose_pub.get_num_connections() > 0 and isinstance(self.inferred_pose, np.ndarray):
            ps = PoseStamped()
            ps.header = Utils.make_header("map")
	    ps.pose.position.x = self.inferred_pose[0]
	    ps.pose.position.y = self.inferred_pose[1]
	    ps.pose.orientation = Utils.angle_to_quaternion(self.inferred_pose[2])
            # FILL OUT THE POSE 
            # Utils.angle_to_quaternion() will probably be useful
            self.pose_pub.publish(ps)

        if self.particle_pub.get_num_connections() > 0:
            if self.MAX_PARTICLES > self.MAX_VIZ_PARTICLES:
                # randomly downsample particles to avoid killing RViz with tons of particles
                proposal_indices = np.random.choice(np.arange(self.MAX_PARTICLES), self.MAX_VIZ_PARTICLES)
                # proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES)
                self.publish_particles(self.particles[proposal_indices,:])
            else:
                self.publish_particles(self.particles)

        if self.pub_fake_scan.get_num_connections() > 0 and isinstance(self.ranges, np.ndarray):
            # generate the scan from the point of view of the inferred position for visualization
            # this should always align with the map, if not then you are probably using RangeLibc wrong 
            # or you have a coordinate space issue
            self.viz_queries[:,0] = self.inferred_pose[0]
            self.viz_queries[:,1] = self.inferred_pose[1]
            self.viz_queries[:,2] = self.laser_angles + self.inferred_pose[2]
            self.range_method.calc_range_many(self.viz_queries, self.viz_ranges)
            self.publish_scan(self.laser_angles, self.viz_ranges)

    def publish_particles(self, particles):
        pa = PoseArray()
        pa.header = Utils.make_header("map")
        pa.poses = Utils.particles_to_poses(particles)
        self.particle_pub.publish(pa)

    def publish_scan(self, angles, ranges):
        ls = LaserScan()
        ls.header = Utils.make_header("laser", stamp=self.last_stamp)
        ls.angle_min = np.min(angles)
        ls.angle_max = np.max(angles)
        ls.angle_increment = np.abs(angles[0] - angles[1])
        ls.range_min = 0
        ls.range_max = np.max(ranges)
        ls.ranges = ranges
        self.pub_fake_scan.publish(ls)

    def lidarCB(self, msg):
        if not isinstance(self.laser_angles, np.ndarray):
            print "...Received first LiDAR message"
            # YOUR CODE - initialize any cached numpy arrays, likely including the following
            self.laser_indeces = np.arange(0,len(msg.ranges),len(msg.ranges)/50) #52 scans 
	    self.laser_angles = np.arange(msg.angle_min,msg.angle_max+msg.angle_increment,msg.angle_increment,dtype=np.float32)[self.laser_indeces]
            self.viz_queries = np.zeros((len(self.laser_angles),3),dtype=np.float32)
            self.viz_ranges = np.zeros(len(self.laser_angles),dtype=np.float32)

        # store anything used in MCL update
	self.last_stamp =  msg.header.stamp 
        self.lasar_ranges = np.array(msg.ranges,dtype=np.float32)[self.laser_indeces]
        self.lidar_initialized = True



    # Odometry data is accumulated via dead reckoning, so it is very inaccurate
    # this function determines relative shift in the coordinate space of the car
    # on the simulator, the odometry data is perfect state information
    def odomCB(self,msg):
	#figure out the delta x and delta y from the last update
        delta_x = msg.pose.pose.position.x - self.oldOdomX
	delta_y = msg.pose.pose.position.y - self.oldOdomY
	
	#get what the odometry believed was the motion angle off of the pointing angle
	phi = self.oldOdomTheta-math.atan(delta_y/(delta_x+.000001))
	
	#find the distance traveled
	hyp = (delta_x**2+delta_y**2)**.5
	
	#corrections for when we are in reverse
	if (delta_x<0 and delta_y<0) or (delta_x<0 and delta_y>0): hyp = -hyp

	#convert motion to local cordinates
        local_delta_x = math.cos(phi)*hyp
	local_delta_y = -1*math.sin(phi)*hyp
	
	#get the new_theta and find delta_theta
        new_theta = Utils.quaternion_to_angle(msg.pose.pose.orientation) 
        delta_theta = new_theta - self.oldOdomTheta

	#update the action atribute
        self.action = [local_delta_x,local_delta_y,delta_theta]

	#update for next time we call so we can find the deltas
        self.oldOdomTheta = new_theta
        self.oldOdomX = msg.pose.pose.position.x
        self.oldOdomY = msg.pose.pose.position.y
        
        # this topic is slower than lidar, so you may want to do MCL update in response to this callback
        self.update()

    def clicked_pose(self, msg):
        # if we get a click make a new set of particles
        if type(msg) == PoseWithCovarianceStamped:
	    x = msg.pose.pose.position.x
	    y = msg.pose.pose.position.y
	    angle = Utils.quaternion_to_angle(msg.pose.pose.orientation)
	    self.particles[:] = np.array([x,y,angle])
	    #make all the particles the position give
	else:
	    x = msg.point.x
	    y = msg.point.y
	    self.particles[:] = np.array([x,y,0])
	    #dont know angle so just put random angles
	    self.particles[:,2] = np.random.rand(self.MAX_PARTICLES)*2*math.pi
	    
	#now add noise since we dont trust the clicks
	self.particles += np.array(np.random.normal(0.0, 0.05, self.particles.shape))

    def precompute_sensor_model(self):
        print "Precomputing sensor model"
        table_width = int(self.MAX_RANGE_PX) + 1

        #meshgrid of data
        (x,y) = np.meshgrid(np.linspace(0,self.MAX_RANGE_PX,table_width),np.linspace(0,self.MAX_RANGE_PX,table_width))
        #normal along identity
        z = 2*norm.pdf(x, y,5)
        #uniform
        z += 2.0/self.MAX_RANGE_PX
        #ramp
        for row in xrange(table_width):
            z[row][0:row] += .01 - .01*np.arange(row,dtype=np.float32)/row
        #max_dist
        z[:,-1:] = .3
        #normalize
        #for i in range(len(z)):
        #    z[i]= z[i]/sum(z[i])
        z/z.sum(axis=1, keepdims=True)
        #transpose and save it had to use ascontiguousarray for cpython to be happy
        self.sensor_model_table = np.ascontiguousarray(z.T)
        # upload the sensor model to RangeLib for ultra fast resolution later
        self.range_method.set_sensor_model(self.sensor_model_table)

        # code to generate visualizations of the sensor model
        if False:
            # visualize the sensor model
            fig = plt.figure()
            ax = fig.gca(projection='3d')

            # Make data.
            X = np.arange(0, table_width, 1.0)
            Y = np.arange(0, table_width, 1.0)
            X, Y = np.meshgrid(X, Y)

            # Plot the surface.
            surf = ax.plot_surface(X, Y, self.sensor_model_table, cmap="bone", rstride=2, cstride=2,
                                   linewidth=0, antialiased=True)

            ax.text2D(0.05, 0.95, "Precomputed Sensor Model", transform=ax.transAxes)
            ax.set_xlabel('Ground truth distance (in px)')
            ax.set_ylabel('Measured Distance (in px)')
            ax.set_zlabel('P(Measured Distance | Ground Truth)')

            plt.show()
        elif False:
            plt.imshow(self.sensor_model_table * 255, cmap="gray")
            plt.show()
        elif False:
            plt.plot(self.sensor_model_table[:,140])
            plt.plot([139,139],[0.0,0.08], label="test")
            plt.ylim(0.0, 0.08)
            plt.xlabel("Measured Distance (in px)")
            plt.ylabel("P(Measured Distance | Ground Truth Distance = 140px)")
            plt.show()

    # proposal dist should be N by 3 numpy array, action is probably a size 3 numpy vector
    def motion_model(self):
    	#precompute errors
	random_error = np.random.normal(0.0, 0.05, (len(self.particles),2))
	random_error_ang = np.random.normal(0.0, 0.05, (len(self.particles)))
	#get the trigonometry features for each particle
        particle_cos = np.cos(self.particles[:,2])
        particle_sin = np.sin(self.particles[:,2])
        
        #update the particles and use scaled (to velocity) normal noise
        self.particles[:,0] += (self.action[0]*particle_cos - self.action[1]*particle_sin)*np.random.normal(1.0, 0.01, (len(self.particles)))
        self.particles[:,1] += (self.action[0]*particle_sin + self.action[1]*particle_cos)*np.random.normal(1.0, 0.01, (len(self.particles)))
        self.particles[:,2] += self.action[2] * np.random.normal(1.0, 0.01, (len(self.particles)))
        #now just add some normal noise as well
        self.particles[:,:2]+= random_error
	self.particles[:,2] += random_error_ang

    def sensor_model(self):
        # only allocate buffers once to avoid slowness
        if self.first_sensor_update:
            self.ranges = np.zeros(len(self.particles)*len(self.laser_angles), dtype=np.float32)
            self.first_sensor_update = False

	#self.queries = np.repeat(self.particles,len(self.laser_angles),axis=0)
        self.range_method.calc_range_repeat_angles(self.particles, self.laser_angles, self.ranges)
	self.range_method.eval_sensor_model(self.lasar_ranges, self.ranges, self.weights, len(self.laser_angles), len(self.particles))

    # this function is on the critical path
    def MCL(self):
    	#run both our models
	self.motion_model()
	self.sensor_model()
	
	#normalize the weights
	self.weights /= self.weights.sum()
	
	#select the particles that we want to keep
	safe_particles = np.random.choice(np.arange(self.MAX_PARTICLES), self.MAX_PARTICLES, p=self.weights)
        self.particles = self.particles[safe_particles,:]
        
        #now the weights should be uniform again
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES) 

    # returns the expected value of the pose given the particle distribution
    def expected_pose(self):
    	#calculated weighted average
	self.inferred_pose = np.matmul(self.particles.T,self.weights)
	#since angles are mod 360 we could get into an error where hald the particles are at 0 and the other are at 360
	#so the mean would be 180 which is completly wrong this fixes that by making sure the angle is on one of the particles
	self.inferred_pose[2] = min(self.particles[:,2],key=lambda x:abs(x-self.inferred_pose[2]))


    def update(self):
        if self.lidar_initialized and self.map_initialized:
            if self.state_lock.locked():
                print "Concurrency error avoided"
            else:
                self.state_lock.acquire()
                #this runs each of the processes
                self.MCL()
                self.expected_pose()
		self.publish_tf()
		self.visualize()
                self.state_lock.release()

if __name__=="__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFiler()
    rospy.spin()
