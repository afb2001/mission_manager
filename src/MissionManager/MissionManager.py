#!/usr/bin/env python
"""
Created on Mon Nov 20 12:26:52 2017

@author: field
"""

import rospy
import MissionPlan.missionplan
import geodesy.utm
import project11_transformations
from geometry_msgs.msg import PoseStamped 
from geographic_msgs.msg import GeoPointStamped
from project11_transformations.srv import *

class MissMan_Node():
    
    def __init__(self):
        rospy.init_node('MissionManager')
        self.Mission = MissionPlan.missionplan.Mission()
        #print self.Mission;
        
        # Receive up-to-date UTM coordinates of boat
        rospy.Subscriber('/position_utm', PoseStamped, self.position_utm_callback, queue_size = 1)
        
        # Receive up-to-date lat/lon coordinates of boat
        rospy.Subscriber('/position', GeoPointStamped, self.position_callback, queue_size = 1)
        
        # TODO:
        # Subscribe to waypoints/behaviors from AutonomousMissionPlanner. Not sure what topic.
        
        pass
   
    def position_utm_callback(self, position_msg):
        pass
    
    def position_callback(self, position_msg):
        # Use Roland's service to convert to earth-centered, earth-fixed.
        # Converts GeoPointStamped to PointStamped
    
        rospy.wait_for_service('wgs84_to_earth')
        
        wgs84_to_earth = rospy.ServiceProxy('wgs84_to_earth', LatLongToEarth)
        
        ecef = wgs84_to_earth(position_msg)
        #ecef = self.lat_long_to_ecef(position_msg)
        self.ecef_x = ecef.earth.point.x
        self.ecef_y = ecef.earth.point.y
        
        print "***POSITION MSG: "
        print position_msg
        print "***ECEF: "
        print ecef
        
        pass
    
    def lat_long_to_ecef(self, geo_point_stamped):
        # Use Roland's service to convert to earth-centered, earth-fixed.
        # Converts GeoPointStamped to PointStamped
        rospy.wait_for_service('wgs84_to_earth')
        wgs84_to_earth = rospy.ServiceProxy('wgs84_to_earth', LatLongToEarth)
        
        # Return message of type PointStamped        
        return wgs84_to_earth(geo_point_stamped)
        
        
    
    def readMission(self, filename): 
        '''Read mission file and make list of nav objectives'''
        self.Mission.fromfile(filename)
        #print file
        pass
   
    
    def extractPosition(self, wpt):
        '''Helper method to retrieve position from input mission file'''
        #Get Position                
        lat = wpt["nav"]["position"]["latitude"]
        lon = wpt["nav"]["position"]["longitude"]
        alt = wpt["nav"]["position"]["altitude"]        
        
        position = {"lat" : lat, "lon" : lon, "alt" : alt}        
        #position = GeoPointStamped()
        #position.position.latitude = lat
        #position.position.longitude = lon
        #position.position.altitude = alt
        
        # Return GeoPointStamped message
        return position
        
        
    def extractOrient(self, wpt):
        '''Helper method to retrieve orientation from input mission file'''
        #Get Orientation
        head = wpt["nav"]["orientation"]["heading"]
        roll = wpt["nav"]["orientation"]["roll"] 
        pitch = wpt["nav"]["orientation"]["pitch"]
        
        orient = {"head" : head, "roll" : roll, "pitch" : pitch}
        
        return orient
        
        
    def headToYaw(self, heading):
        '''Helper method to convert heading (0 degrees north, + rotation clockwise)
        to yaw (0 degrees east, + rotation coutner-clockwise)'''
       
        if type(heading) == int \
        or type(heading) == float \
        or type(heading) == long: 
            if heading > 360: 
                x = int(heading / 360)
                heading = heading - (x * 360)
        
            temp = (heading * (-1)) + 90
            if temp < 0: 
                temp = temp + 360
            return temp
        
        else:
            return heading
        
       
    def convertToRobotCoordinates(self, position, orient):
        # Convert to UTM
        utm = geodesy.utm.fromLatLong(position["lat"], position["lon"])
        
        # Convert to PoseStamped() message type.
        pos = PoseStamped()
        
        # Double-check if correct!
        pos.pose.position.x = utm.easting
        pos.pose.position.y = utm.northing
        pos.pose.position.z = utm.altitude
        
        # Double-check if correct!
        # Do we need to convert heading from 0 deg North to 0 deg East?
        #Heading rotates aroung z-axis
        pos.pose.orientation.z = self.headToYaw(orient["head"])
        #Roll rotates around x-axis
        pos.pose.orientation.x = orient["roll"]
        #Pitch rotates around y-axis
        pos.pose.orientation.y = orient["pitch"]
        
        #Currently returns UTM - Must determine (0,0) point to return robot coordinates
        
        #latOrigin and lonOrigin should be assigned (0,0) point in robot coordinates
        #latRobotCoord = utm.easting - latOrigin
        #lonRobotCoord = utm.northing - lonOrigin
        #pos.pose.position.x = latRobotCoord
        #pos.pose.position.y = lonRobotCoord
        
        x = GeoPointStamped()
        
        rospy.wait_for_service('wgs84_to_earth')
        
        wgs84_to_earth = rospy.ServiceProxy('wgs84_to_earth', LatLongToEarth)
        test = wgs84_to_earth(x)
        print test
        print pos
    
    
    def setNextWaypoint(self):
        ''' Send waypoint (in robot coordinates) to MOOS.
        topic: /moos/wpt_updates
        String: points=x1,y1:x2,y2
        '''
        pass
    
    def getDistanceToNextWaypoint(self):
        #Must subscribe to current position updates to determine
        pass
    
    def getTimeToNextWaypoint(self):
        #Must subscribe to current position and speed to determine
        pass
    
    def start():
        pass        
        
    def pause():
        pass        
        
    def abort():
        pass
    
    def run(self):
        #Read mission plan output by missionplan.py
        #This is only a sample mission plan and will need to be replaced
        self.readMission('/home/monster-kitty/project11/catkin_ws/src/mission-plan/src/MissionPlan/mission.txt')
        
        #For testing only!        
        #self.Mission in this test is default missionplan.py output        
        print self.Mission;
    
        #For testing only!
        #In default mission plan, len(self.Mission.plan["NAVIGATION"]) is 2
        #One for waypoint; one for path
        #print len(self.Mission.plan["NAVIGATION"])
        
        #For each item in NAVIGATION:
        for x in range(0, len(self.Mission.plan["NAVIGATION"])):
        
            #Determine whether waypoint or path:
            
            if self.Mission.plan["NAVIGATION"][x].has_key("waypoint"):
                wpt =  self.Mission.plan["NAVIGATION"][x]["waypoint"]
                position = self.extractPosition(wpt)
                orient = self.extractOrient(wpt)
                
                print "TEST WPT: ", self.convertToRobotCoordinates(position, orient)
               
                utm = geodesy.utm.fromLatLong(position["lat"], position["lon"])
                #print "utm: ", utm
            
            elif self.Mission.plan["NAVIGATION"][x].has_key("path"):
                path = self.Mission.plan["NAVIGATION"][x]["path"]
                # print path
                for item in path["nav"]:
                    # print "TEST b"                    
                    # print item
                    if item.has_key("waypoint"):
                        wpt = item["waypoint"]
                        position = self.extractPosition(wpt)
                        orient = self.extractOrient(wpt)
            
                        print "TEST PATH: ", self.convertToRobotCoordinates(position, orient)
                         
                        utm = geodesy.utm.fromLatLong(position["lat"], position["lon"])
                        #print "utm: ", utm
            
           # self.convertToRobotCoordinates()
            
            self.setNextWaypoint()
            
            distance_to_go = self.getDistanceToNextWaypoint()
            
            self.getTimeToNextWaypoint()
            
            #while distance_to_go > 3.0:            
            while False:            
                distance_to_go = self.getDistanceToNextWaypoint()            
                self.getTimeToNextWaypoint()
                # Report distance and time to waypoint. TBD
            
            
     
    
    
    
    
