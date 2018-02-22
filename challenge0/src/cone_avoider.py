#!/usr/bin/env python
import rospy
import math
import numpy as np
import time


class ConeAvoider():
    def __init__(self):
	print "worked"


if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("Cone_Navigator_Node")

    # Init the node
    ConeNavigator()

    # Don't let this script exit while ROS is still running
    rospy.spin()

