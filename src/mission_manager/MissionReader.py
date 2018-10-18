#!/usr/bin/env python
"""
Created on Mon Nov 20 12:26:52 2017

@author: ldavis

getWaypointsAndBehaviors() function reads 
mission plan from Autonomous mission planner. 
Returns "masterWaypointAndBehaviorArray" 
containingdefault behaviors, waypoints, and
waypoint-associated behaviors.
"""

import rospy
import json
import mission_plan.missionplan
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import String
from project11_transformations.srv import LatLongToEarth


class MissionReader():
    
    def __init__(self):
        
        # Create instance of missionplan
        # Used to read mission.txt file or /mission_plan subscriber...
        # And store mission plan.
        self.Mission = mission_plan.missionplan.Mission()
        self.missionPlanData = False
        
        # Array to contain waypoints and behaviors.
        self.waypointBehaviorArray = []
        
        # Variable for waypoint - in map / robot coordinates
        #self.waypoint = PointStamped()   
        
        # Variable for current position of boat--to be set in position_callback
        self.currPosBoat_LatLon = GeoPointStamped()
        self.currPosBoat_ECEF = PointStamped()
        self.currPosBoat_Map = PointStamped()
        
        # Receive up-to-date mission from Autonomous Mission Planner 
        rospy.Subscriber('/mission_plan', String, self.mission_plan_callback, queue_size = 1)
        
    
    def mission_plan_callback(self, missionPlan_msg):
        
        #print "In mission plan callback."
        # Assign self.Mission.plan to be incoming mission plan from /mission_plan.
        self.readMission(missionPlan_msg.data)
        #print "self.Mission.plan", self.Mission.plan
        print "missionPlan_msg", missionPlan_msg.data
        self.missionPlanData = True
        #print self.missionPlanData
        
    
    def readMission(self, filename): 
        '''Read mission file and make list of nav objectives'''
        print "Mission received. Reading mission..."
        #self.Mission.fromfile(filename)
        self.Mission.fromString(filename)
        pass    
    
    
    def extractPosition(self, wpt):
        '''Helper method to retrieve position from input mission file'''
        position = GeoPointStamped()
        
        position.position.latitude = wpt["nav"]["position"]["latitude"]
        position.position.longitude = wpt["nav"]["position"]["longitude"]
        
        # Altitude may be "None" -- GeoPointStamped requires float
        if type(wpt["nav"]["position"]["altitude"]) is float:
            position.position.altitude = wpt["nav"]["position"]["altitude"]
        
        # Return GeoPointStamped message
        return position
        
    
    def lat_long_to_ecef(self, geo_point_stamped):
        # Uses Roland's service to convert to earth-centered, earth-fixed.
        # Converts GeoPointStamped to PointStamped
        rospy.wait_for_service('wgs84_to_earth')
        wgs84_to_earth = rospy.ServiceProxy('wgs84_to_earth', LatLongToEarth)
        
        # Return message of type PointStamped with frame_id: "earth"
        return wgs84_to_earth(geo_point_stamped)
        
        
    def convertToTF2PointStamped(self, old):
        '''
        This is a dumb function to convert from a normal 
        geometry_msgs PointStamped message (returned from lat_long_to_earth)
        to a dumb tf2_geometry_msgs PointStamped message and I'm mad about it.
        '''
        new = PointStamped()
        
        new.header.seq = old.earth.header.seq
        new.header.stamp.secs = old.earth.header.stamp.secs
        new.header.stamp.nsecs = old.earth.header.stamp.nsecs
        new.header.frame_id = old.earth.header.frame_id
        new.point.x = old.earth.point.x
        new.point.y = old.earth.point.y
        new.point.z = old.earth.point.z
        
        return new
        
        
    def convertToMapCoordinates(self, ecef):
        '''Uses tf transform to convert from ECEF to local map coordinates.'''
        
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        
        # This step is necessary to wait until transform becomes available
        while not buffer.can_transform('map', 'earth', rospy.Time(0)):
            print "Waiting for ECEF to Map transformation..."
            rospy.sleep(0.5)
            if rospy.is_shutdown:
                break
        
        map = buffer.transform(self.convertToTF2PointStamped(ecef), "map")
       
        return map      
    
    
    def getWaypointsAndBehaviors(self):
        '''
        Extracts waypoints and behaviors from /mission_plan topic.
        
        Returns masterWaypointAndBehaviorArray arranged as follows:
        masterWaypointAndBehaviorArray = [[defaultBehaviorArray], ...
        [type, [[waypoint], [behaviors]]], [type, [[waypoint], [behaviors]]]]
        '''
        
        if not self.missionPlanData:
            print "Mission plan not yet available..."
            while not self.missionPlanData:    
                #print "In wait loop."
                #print self.missionPlanData
                rospy.sleep(0.5)
                #rospy.spin()
                if rospy.is_shutdown():
                   break
       
        masterWaypointAndBehaviorArray = []
        
        if self.missionPlanData:
            
            defaultBehaviorArray = []
            #wptBehaviors = []
            #wptBehaviorPairs = []
            masterWaypointAndBehaviorArray = []
            typeArray = [None]
           
            # TODO: Have Roland reincorporate DEFAULT PARAMETERS block into /mission_plan
            # Determine and set default behaviors
            if self.Mission.plan.has_key("DEFAULT_PARAMETERS"):
                if self.Mission.plan["DEFAULT_PARAMETERS"].has_key("behaviors"):
                    # TODO: Double check that this is an array:
                    defaultBehaviorArray.insert(0, self.Mission.plan["DEFAULT_PARAMETERS"]["behaviors"])
                    
# # # # # # # # # # # # # # TODO: DUMMY TO BE REPLACED # # # # # # # # # # # # # # # #
                  
            dummy = '/home/monster-kitty/project11/catkin_ws/src/missionmanager/dummyDontRunAground.txt'
            with open(dummy,'r') as infile:            
                tempDefault = json.load(infile)
            if tempDefault.has_key("DEFAULT_PARAMETERS"):
                if tempDefault["DEFAULT_PARAMETERS"].has_key("behaviors"):
                    defaultBehaviorArray.insert(0, tempDefault["DEFAULT_PARAMETERS"]["behaviors"])
            
# # # # # # # # # # # # # # DUMMY TO BE REPLACED - END # # # # # # # # # # # # # # # # 
            
            wptBehaviorPairs = [[],defaultBehaviorArray]
            typeArray.append(wptBehaviorPairs)
            masterWaypointAndBehaviorArray.insert(0, typeArray)
           
            mpNavigation = self.Mission.plan["NAVIGATION"]
            #For each item in NAVIGATION:
            for x in range(0, len(mpNavigation)):
                typeArray = [None]
               
                # If a single waypoint, execute the following...            
                if mpNavigation[x].has_key("waypoint"):
                    # TODO: Extract type (survey line, turn, etc) and append to master array at end.
                    # I think it's unlikely that a straight-up waypoint will have a type.
                    #if mpNavigation["waypoint"].has_key("type"):                        
                    #     typeArray = path["type"]
                    #typeArray = [None]
               
                    wptBehaviors = []                    
                    # TODO: Extract waypoint-specific behaviors and create array
                    # TODO: Double check that this generates an array... I think it does...
                    if mpNavigation[x]["waypoint"].has_key("behaviors"):
                        wptBehaviors = mpNavigation[x]["waypoint"]["behaviors"]
                    
                    wpt =  mpNavigation[x]["waypoint"]
                    # GeoPointStamped message with waypoint latitude, longitude, and altitude
                    position = self.extractPosition(wpt)
                    waypoint = self.convertToMapCoordinates(self.lat_long_to_ecef(position))
                    waypointArray = [waypoint.point.x, waypoint.point.y]
                   
                    wptBehaviorPairs = [waypointArray, wptBehaviors]
                    typeArray.append(wptBehaviorPairs)
                    masterWaypointAndBehaviorArray.append(typeArray)
                
                elif mpNavigation[x].has_key("path"):
                    path = mpNavigation[x]["path"]
                    
                    # TODO: Extract type (survey line, turn, etc) and append to master array at end.
                    if path.has_key("type"):                        
                        typeArray = [path["type"]]
                    
                    
                    for y in range(0, len(path["nav"])):
                        if path["nav"][y].has_key("waypoint"):
                            
                            wptBehaviors = []
                            # TODO: Extract waypoint-specific behaviors and create array
                            # TODO: Double check that this generates an array... I think it does...
                            if path["nav"][y]["waypoint"].has_key("behaviors"):
                                wptBehaviors = path["nav"][y]["waypoint"]["behaviors"]
                            
                            wpt = path["nav"][y]["waypoint"]
                            # GeoPointStamped message with waypoint latitude, longitude, and altitude
                            position = self.extractPosition(wpt)
                            waypoint = self.convertToMapCoordinates(self.lat_long_to_ecef(position))
                            waypointArray = [waypoint.point.x, waypoint.point.y]
                            
                            wptBehaviorPairs = [waypointArray, wptBehaviors]
                            typeArray.append(wptBehaviorPairs)
                    masterWaypointAndBehaviorArray.append(typeArray)
        
            #defaultBehaviors = masterWaypointAndBehaviorArray[0][1][1]        
            #print "defaultBehaviors", defaultBehaviors           
            return masterWaypointAndBehaviorArray
