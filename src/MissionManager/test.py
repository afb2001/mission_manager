#!/usr/bin/env python

"""
Created on Fri Jul 27 14:15:25 2018

@author: monster-kitty
"""

import rospy
import json
import MissionPlan.missionplan
import MissionReader
import DontRunAgroundBen
import geodesy.utm
import project11_transformations
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import String, Float32, Int32
from missionmanager.msg import BehaviorControl
from project11_transformations.srv import LatLongToEarth
import math


     
class Test():
   
    def __init__(self, default = True):
        rospy.init_node("test_node")
    
    def run(self):
        reader = MissionReader.MissionReader()
        array = reader.getWaypointsAndBehaviors()
        defaultBehaviors = array[0][1][1]
        print "defaultBehaviors:", defaultBehaviors
        
        for x in range(1, len(array)):
            # Get type:
            type = array[x][0]
            print "Type:", type
            
            for y in range(1, len(array[x])):
                # Get waypoint
                waypoint = array[x][y][0]
                print "Waypoint:", waypoint
                # Get behavior
                behavior = array[x][y][1]
                print "Behavior:", behavior
        pass
    
if __name__ == '__main__':
     a = Test()
     a.run()    