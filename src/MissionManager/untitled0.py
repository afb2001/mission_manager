#!/usr/bin/env python
"""
Created on Mon Nov 20 12:26:52 2017

@author: field
"""

import rospy
import MissionPlan.missionplan
import geodesy.utm
import project11_transformations
import tf2_ros
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from std_msgs.msg import String
from project11_transformations.srv import LatLongToEarth
import math

class MissMan_Node():
    
    
    
    def __init__(self):
        rospy.init_node('MissionManager')
        self.Mission = MissionPlan.missionplan.Mission()
        #print self.Mission;
        
        # Variable for waypoint
        self.waypoint = PointStamped()   
        
        # Variable for current position of boat--to be set in position_callback
        self.currPosBoat_LatLon = GeoPointStamped()
        self.currPosBoat_ECEF = PointStamped()
        self.currPosBoat_Map = PointStamped()
        
        # Set minimum distance to waypoint
        self.threshold = 3.0
        
        # Receive up-to-date UTM coordinates of boat
        rospy.Subscriber('/position_utm', PoseStamped, self.position_utm_callback, queue_size = 1)
        
        # Receive up-to-date lat/lon coordinates of boat
        rospy.Subscriber('/position', GeoPointStamped, self.position_callback, queue_size = 1)
        
        # To publish waypoint updates to MOOS
        self.wpt_updates = rospy.Publisher('/moos/wpt_updates', String, queue_size = 10)
        # TODO:
        # Subscribe to waypoints/behaviors from AutonomousMissionPlanner. Not sure what topic.
        
        pass
   
    def position_utm_callback(self, position_msg):
        pass
    
    def position_callback(self, position_msg):
        # Save lat/lon coordinates
        self.currPosBoat_LatLon = position_msg        
        
        # Use Roland's service (in lat_long_to_ecef) 
        # to convert to earth-centered, earth-fixed.
        # Converts GeoPointStamped to PointStamped

        self.currPosBoat_ECEF = self.lat_long_to_ecef(position_msg)
        
        # Convert from geometry msgs PointStamped (ECEF) to TF2 PointStamped (ECEF)
        # tempTF2_PointStamped = self.convertToTF2PointStamped(self.currPosBoat_ECEF)
        # NOTE: convertToMapCoordinates will call convertToTF2PointStamped
        
        # Convert from TF2 PointStamped (ECEF) to geometry msgs  PointStamped (Map Coordinates)
        self.currPosBoat_Map = self.convertToMapCoordinates(self.currPosBoat_ECEF)
        
            
    def lat_long_to_ecef(self, geo_point_stamped):
        # Use Roland's service to convert to earth-centered, earth-fixed.
        # Converts GeoPointStamped to PointStamped
        rospy.wait_for_service('wgs84_to_earth')
        wgs84_to_earth = rospy.ServiceProxy('wgs84_to_earth', LatLongToEarth)
        
        # Return message of type PointStamped with frame_id: "earth"
        return wgs84_to_earth(geo_point_stamped)
        
            
    def readMission(self, filename): 
        '''Read mission file and make list of nav objectives'''
        self.Mission.fromfile(filename)
        #print file
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
        
    """
    def extractPosition_ECEF(self, wpt):
        '''Helper method to retrieve position from input mission file'''
        position = GeoPointStamped()
        
        position.position.latitude = wpt["nav"]["position"]["latitude"]
        position.position.longitude = wpt["nav"]["position"]["longitude"]
        
        # Altitude may be "None" -- GeoPointStamped requires float
        if type(wpt["nav"]["position"]["altitude"]) is float:
            position.position.altitude = wpt["nav"]["position"]["altitude"]
        
        position.header.frame_id = 'ecef'
        # Return GeoPointStamped message
        return position
     """  
        
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
        
    """  
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
    """
    
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
        
        #print "*******************TYPE********************"
        #print type(new)
        return new
        
        
    def convertToMapCoordinates(self, ecef):
        '''Uses tf transform to convert from ECEF to local map coordinates.'''
        
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        
        # This step is necessary to wait until transform becomes available
        while not buffer.can_transform('map', 'earth', rospy.Time(0)):
            print "Cannot transform yet!"
            rospy.sleep(0.5)
        
        map = buffer.transform(self.convertToTF2PointStamped(ecef), "map")
        #mapp = buffer.transform(ecef, "map")
        #except:
        
        #print 'ecef'
        #print ecef
        #print 'mapp'
        #print mapp
        #print "type(map)", type(map)
        return map        
        
    
    
    def setNextWaypoint(self, waypoint):
        ''' Send waypoint (in robot coordinates) to MOOS.
        topic: /moos/wpt_updates
        String: points=x1,y1:x2,y2
        '''
        point = "points = %f, %f"%(waypoint.point.x, waypoint.point.y)
        self.wpt_updates.publish(point)
        
    
    def getDistanceToNextWaypoint(self):
        #Must subscribe to current position updates to determine
        boat_x = self.currPosBoat_Map.point.x
        boat_y = self.currPosBoat_Map.point.y
        
        waypoint_x = self.waypoint.point.x
        waypoint_y = self.waypoint.point.y
        
        distance = math.sqrt((boat_x - waypoint_x)**2 + (boat_y - waypoint_y)**2)
        
        #print "distance: ", distance
        return distance
        
    
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
        #print self.Mission;
        
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
                #orient = self.extractOrient(wpt)
                                
                #utm = geodesy.utm.fromLatLong(position["lat"], position["lon"])
                
            
            elif self.Mission.plan["NAVIGATION"][x].has_key("path"):
                path = self.Mission.plan["NAVIGATION"][x]["path"]
                # print path
                for item in path["nav"]:
                    # print "TEST b"                    
                    # print item
                    if item.has_key("waypoint"):
                        wpt = item["waypoint"]
                        position = self.extractPosition(wpt)
                        #orient = self.extractOrient(wpt)
            
                        #utm = geodesy.utm.fromLatLong(position["lat"], position["lon"])
            #print "POSITION: "            
            #print position
           
            #self.convertToTF2PointStamped(self.lat_long_to_ecef(position))           
           
            map = self.convertToMapCoordinates(self.lat_long_to_ecef(position))
            print "map", map
            self.waypoint = map
            
            self.setNextWaypoint(map)
            
            while self.getDistanceToNextWaypoint() > self.threshold:            
                # Report distance and time to waypoint.
                rospy.logout(self.getDistanceToNextWaypoint())            
                rospy.logout(self.getTimeToNextWaypoint())
                
                print "type(self.currPosBoat_ECEF): ", type(self.currPosBoat_ECEF)
                print "self.currPosBoat_Map X", self.currPosBoat_Map.point.x
                print "self.currPosBoat_Map Y", self.currPosBoat_Map.point.y
                print "self.currPosBoat_Lat", self.currPosBoat_LatLon.position.latitude
                print "self.currPosBoat_Lon", self.currPosBoat_LatLon.position.longitude
                print "self.currPosBot_ECEF X", self.currPosBoat_ECEF.point.x
                print "self.currPosBot_ECEF Y", self.currPosBoat_ECEF.point.y
                print "self.currPosBot_ECEF Z", self.currPosBoat_ECEF.point.z
                print "self.waypoint X", self.waypoint.point.x
                print "self.waypoint Y", self.waypoint.point.y
                
                rospy.sleep(1)
            
            
     
    
    
    
    
