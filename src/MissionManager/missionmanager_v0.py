#!/usr/bin/env python
"""
Created on Mon Nov 20 12:26:52 2017

@author: field
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
from std_msgs.msg import String, Float32, Int32, Bool
from missionmanager.msg import BehaviorControl
from project11_transformations.srv import LatLongToEarth
import math
#import MissionManager.Behaviors

class MissMan_Node():
    
    def __init__(self):
        
        rospy.init_node('MissionManager')
        
        # This will be used to read mission.txt file later...
        self.Mission = MissionPlan.missionplan.Mission()
        self.missionReader = MissionReader.MissionReader()
        
        # Variable for waypoint - in map / robot coordinates
        self.waypoint = PointStamped()   
        
        # Variable for current position of boat--to be set in position_callback
        self.currPosBoat_LatLon = GeoPointStamped()
        self.currPosBoat_ECEF = PointStamped()
        self.currPosBoat_Map = PointStamped()
        
        # TODO: Change depth type when we find out what it should be.
        self.depth = Float32
        self.wptIndex = Int32
        
        # TODO: Threshold possibly set thresholds in parameters?
        # Set minimum distance to waypoint
        self.shallowWaterDanger = False
        self.waypointThreshold = 3.0
        self.depthThreshold = 4.0
        self.shallowWaterNegativeCount = 0
        self.shallowWaterPositiveCount = 0
        # If ASV receives this many consecutive negative shallow warnings,
        # reset positive shallowWater counts
        self.shallowWaterNegativeCountThreshold = 5
        # If ASV receives this many consecutive positive shallow warnings, 
        # deviate path and reset negative shallowWater counts
        self.shallowWaterPositiveCountThreshold = 5
        
        #self.xIndex = 1
        self.moos_wptIndex_Update = False
        self.shallowWaterUpdate = False
        self.incrementedLine = False
        
        # Timer used to print status
        self.everyTwoSeconds = rospy.Duration(0.5)        
        
        # Receive up-to-date UTM coordinates of ASV
        rospy.Subscriber('/position_utm', PoseStamped, self.position_utm_callback, queue_size = 1)
        # Receive up-to-date lat/lon coordinates of ASV
        rospy.Subscriber('/position', GeoPointStamped, self.position_callback, queue_size = 1)
        # Receive up-to-date depth 
        rospy.Subscriber('/depth', Float32, self.depth_callback, queue_size = 1)
        #Receive up-to-date warnings from Don't Run Aground, Ben! behavior.
        rospy.Subscriber('/shallow_water_warning', String, self.shallow_water_callback, queue_size = 1)
        # Receive up-to-date waypoint indices
        rospy.Subscriber('/moos/wpt_index', Int32, self.waypoint_index_callback, queue_size = 1)
        # Receive up-to-date waypoint updates
        rospy.Subscriber('/moos/wpt_updates', String, self.waypoint_updates_callback, queue_size = 1)
        
        # To publish waypoint updates to MOOS
        self.wpt_index_publisher = rospy.Publisher('/moos/wpt_index', Int32, queue_size = 10)
        self.wpt_updates_publisher = rospy.Publisher('/moos/wpt_updates', String, queue_size = 10)
        self.behavior_ctrl = rospy.Publisher('/behavior_ctrl', BehaviorControl, queue_size = 10)
        
   
    def position_utm_callback(self, position_msg):
        pass
    
    
    def position_callback(self, position_msg):
        # Save lat/lon coordinates
        self.currPosBoat_LatLon = position_msg        
        
        # Use Roland's service (in lat_long_to_ecef) 
        # to convert to earth-centered, earth-fixed.
        # Converts GeoPointStamped to PointStamped
        self.currPosBoat_ECEF = self.lat_long_to_ecef(position_msg)
        
        # Convert from TF2 PointStamped (ECEF) to geometry msgs PointStamped (Map Coordinates)
        self.currPosBoat_Map = self.convertToMapCoordinates(self.currPosBoat_ECEF)
    
    
    def depth_callback(self, depth_msg):
        # Save up-to-date depth
        self.depth = depth_msg
            
    
    def shallow_water_callback(self, shallow_water_msg):
        
        if shallow_water_msg.data == "EMERGENCY" or shallow_water_msg.data == "WARNING":
            self.shallowWaterDanger = True
                
        elif shallow_water_msg.data == "OK": 
            self.shallowWaterDanger = False
            
        self.shallowWaterUpdate = True
            
    
    def waypoint_index_callback(self, wptIndex_msg):
        # Save up-to-date index
        self.moos_wptIndex = wptIndex_msg.data
        print "moos callback index:", self.moos_wptIndex
        self.moos_wptIndex_Update = True
     
     
    def waypoint_updates_callback(self, wptUpdate_msg):
        pass
        
        
    def readMission(self, filename): 
        '''Read mission file and make list of nav objectives'''
        self.Mission.fromfile(filename)
        pass
    
       
    def lat_long_to_ecef(self, geo_point_stamped):
        # Use Roland's service to convert to earth-centered, earth-fixed.
        # Converts GeoPointStamped to PointStamped
        rospy.wait_for_service('wgs84_to_earth')
        wgs84_to_earth = rospy.ServiceProxy('wgs84_to_earth', LatLongToEarth)
        
        # Return message of type PointStamped with frame_id: "earth"
        return wgs84_to_earth(geo_point_stamped)
    
    """
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
        to yaw (0 degrees east, + rotation counter-clockwise)'''
       
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
            print "Cannot transform yet!"
            rospy.sleep(0.5)
        
        map = buffer.transform(self.convertToTF2PointStamped(ecef), "map")
       
        return map        
       
    
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
    
    
    def printStatus(self):
        # Report distance and time to waypoint.
        print "MOOS waypoint index: ", self.moos_wptIndex
        #print "Line type: ", self.lineType
        print "Distance to next waypoint (m):", self.getDistanceToNextWaypoint()            
        print "Time to next waypoint (s):", self.getTimeToNextWaypoint()
        #print "type(self.currPosBoat_ECEF): ", type(self.currPosBoat_ECEF)
        #print "self.currPosBoat_Map X", self.currPosBoat_Map.point.x
        #print "self.currPosBoat_Map Y", self.currPosBoat_Map.point.y
        #print "self.currPosBoat_Lat", self.currPosBoat_LatLon.position.latitude
        #print "self.currPosBoat_Lon", self.currPosBoat_LatLon.position.longitude
        #print "self.currPosBot_ECEF X", self.convertToTF2PointStamped(self.currPosBoat_ECEF)
        #print "self.currPosBot_ECEF Y", self.convertToTF2PointStamped(self.currPosBoat_ECEF)
        #print "self.currPosBot_ECEF Z", self.convertToTF2PointStamped(self.currPosBoat_ECEF)
        #print "self.waypoint X", self.waypoint.point.x
        #print "self.waypoint Y", self.waypoint.point.y       
        pass
        
    
    def start():
        pass        
        
    def pause():
        pass        
        
    def abort():
        pass
    
    
    def activateBehaviors(self, behaviors):
        # Check to see which behaviors are enabled and activate them.
        #print "activateBehaviors:", behaviors
        #print "type(activateBehaviors):", type(behaviors)
        #print "behaviors[0]:", behaviors[0]
        
        for x in range(len(behaviors)):
            #print "BEHAVIORS", behaviors[x] 
        
            bc = BehaviorControl()
            
            """
            # This behavior doesn't exist yet... So, whatever.
            
            if behaviors[x].has_key("collision_avoidance"):
                bc.Name = "collision_avoidance"
                
                if behaviors[x]["collision_avoidance"]["Active"]:
                    bc.IsActive = True
                    bc.Parameters = json.dumps(behaviors[x])
                    print "COLLISION AVOIDANCE behavior activated!"
                else:
                    bc.IsActive = False
                    print "COLLISION AVOIDANCE behavior deactivated!"
            
                self.behavior_ctrl.publish(bc)
                pass
            """
        
            if behaviors[x].has_key("dont_run_aground_ben"):
                bc.Name = 'dont_run_aground_ben'
                # TODO:
                # Do something...
                if behaviors[x]["dont_run_aground_ben"]["Active"]:
                    bc.IsActive = True
                    bc.Parameters = json.dumps(behaviors[x])
                    print "DON'T RUN AGROUND, BEN behavior being activated..."
                else:
                    bc.IsActive = False
                    print "DON'T RUN AGROUND, BEN behavior being deactivated..."
                
                self.behavior_ctrl.publish(bc)            
                
                # Ensure shallowWaterUpdates are being received before 
                # proceeding with mission plan
                while not self.shallowWaterUpdate:
                    print "Pausing for shallow water updates."
                    rospy.sleep(0.5)
                    if rospy.is_shutdown:
                        break
          
               
    def deactivateBehavior(self, behavior):
        '''
        Input parameter 'behavior' must be string name of behavior. 
        EX: "dont_run_aground_ben"
        '''
        
        bc = BehaviorControl()
        bc.Name = behavior
        bc.IsActive = False
        
        print "Deactivating behavior: ", behavior        
        
        self.behavior_ctrl.publish(bc)
            
        
    def extractWptBhvPairs(self):
        wptBhvPairs = []
            
        for y in range(1, len(self.wptsAndBhvsList[self.xIndex])):
            wptBhvPairs.append(self.wptsAndBhvsList[self.xIndex][y])
        
        return wptBhvPairs
    
    
    def sendToMOOS(self, wptBhvPairs):
        # NOTE: wptBhvPairs (when received from extractWptBhvPairs function) is organized as follows:
        # [[[wpt_x, wpt_y], [wpt_bhv, wpt_bhv, ...]], [[wpt_x, wpt_y], [wpt_bhv, wpt_bhv, ...]]...]
        
        #print "********************", wptBhvPairs    
        print "In send to MOOS."
    
        # Add path/waypoint(s) to list to be sent to MOOS
        pointMsg = String()
        point = "points = "
        
        for x in range(len(wptBhvPairs)):
            tempXY = "%f, %f"%(wptBhvPairs[x][0][0], wptBhvPairs[x][0][1])
            point = point + tempXY
            
            if x < (len(wptBhvPairs) - 1):
                point = point + " : "
        
        pointMsg.data = point
        
        print "POINT:", pointMsg
        self.moos_wptIndex_Update = False
        # Publish path/waypoint(s) to MOOS
        self.wpt_updates_publisher.publish(pointMsg)
        
        
    def incrementSurveyLine(self):
        print "In increment survey line"
        self.incrementedLine = True
        
        tempX = self.xIndex
        #print "self.xIndex", self.xIndex
        print "increment: tempX: ", tempX
        print "increment: len(self.wptsAndBhvsList): ", len(self.wptsAndBhvsList)
        
        if tempX < (len(self.wptsAndBhvsList) - 1):
            tempX += 1
            lineType = self.wptsAndBhvsList[tempX][0]
            print "lineType: ", lineType
            
            while tempX < (len(self.wptsAndBhvsList) - 1) and lineType != 'survey_line':
                tempX += 1
                lineType = self.wptsAndBhvsList[tempX][0]
                print "lineType: ", lineType
                
        if tempX >= (len(self.wptsAndBhvsList) - 1) and lineType != 'survey_line': 
            print "No remaining survey lines; stop ASV or take other corrective action immediately."
            return
        else:
            self.xIndex = tempX
            print "increment: self.xIndex = tempX: ", self.xIndex
            self.shallowWaterDanger = False
            wptBhvPairs = self.extractWptBhvPairs()
            print "increment: wptBhvPairs: ", wptBhvPairs
            #self.setNextWaypoint(wptBhvPairs)
            self.sendToMOOS(wptBhvPairs)
            
            #self.moos_wptIndex_Update = False
            currix = String()
            currix.data = "currix = 1"
            self.wpt_updates_publisher.publish(currix)
            
            #self.incrementedLine = True
            
            finalIndex = (len(wptBhvPairs) - 1)
            self.waypoint.point.x = wptBhvPairs[finalIndex][0][0]
            self.waypoint.point.y = wptBhvPairs[finalIndex][0][1]
            
            # TODO: Ignoring waypoint behaviors for now. Fix later.
            # Think about this...
            while self.getDistanceToNextWaypoint() > self.waypointThreshold:
                #if self.incrementedLine and not self.shallowWaterDanger:
                self.printStatus()
                    
                if rospy.is_shutdown():
                    break
            
            # When breaking out of previous loop, reactivate behavior
            self.activateBehaviors(self.defaultBehaviors)
  

    def setNextWaypoint(self, wptBhvPairs):
        print "In setNextWaypoint."
        # NOTE: wptBhvPairs (when received from extractWptBhvPairs function) is organized as follows:
        # [[[wpt_x, wpt_y], [wpt_bhv, wpt_bhv, ...]], [[wpt_x, wpt_y], [wpt_bhv, wpt_bhv, ...]]...]
        
        self.sendToMOOS(wptBhvPairs)
        
# # # # # NEW - This ignores waypoint behaviors!
        finalIndex = (len(wptBhvPairs) - 1)
        self.waypoint.point.x = wptBhvPairs[finalIndex][0][0]
        self.waypoint.point.y = wptBhvPairs[finalIndex][0][1]
        # TODO: Ignoring waypoint behaviors for now. Fix later.
        while self.getDistanceToNextWaypoint() > self.waypointThreshold:
            if self.shallowWaterDanger: # and not self.incrementedLine:
                    print "SHALLOW WATER WARNING ACTIVATED"
                    # Suspend dontRunAgroundBen behavior and increment survey line
                    self.deactivateBehavior("dont_run_aground_ben")
                    self.shallowWaterDanger = False
                    self.incrementSurveyLine()
                    return
                
            # TODO: Come up with a better way to handle shutdowns...
            if rospy.is_shutdown():
                break
# # # # # # # # # # # # # # # # # # # # # # # # #
     
     
    def runOnce(self):
        #Read mission plan output by missionplan.py
        #This is only a sample mission plan and will need to be replaced
        #self.readMission('/home/monster-kitty/project11/catkin_ws/src/missionmanager/mission_wpt_path.txt')
        
        print "Running mission manager."
        
        # Get mission plan from Autonomous Mission Planner
        # self.wptsAndBhvsList organized as follows:
        # 0: [-none-,{[-none-, -none-], [dflt_bhv, dflt_bhv, ...]}]
        # 1: [type,{[map_wpt_x, map_wpt_y], [wpt_bhv, wpt_bhv, ...]}, {[map_wpt_x, map_wpt_y], [wpt_bhv, wpt_bhv, ...]}, ...]
        # 2: [type,{[map_wpt_x, map_wpt_y], [wpt_bhv, wpt_bhv, ...]}, {[map_wpt_x, map_wpt_y], [wpt_bhv, wpt_bhv, ...]}, ...]
        # ...        

        self.wptsAndBhvsList = self.missionReader.getWaypointsAndBehaviors()
        
        # Set default behaviors
        self.defaultBehaviors = self.wptsAndBhvsList[0][1][1]
        self.activateBehaviors(self.defaultBehaviors)
        
        self.xIndex = 1
        
        while self.xIndex < len(self.wptsAndBhvsList):
            self.incrementedLine = False
            print "runOnce: len(self.wptsAndBhvsList):", len(self.wptsAndBhvsList)
            print "runOnce: self.xIndex: ", self.xIndex
            # Get line type for first line:
            #self.lineType = self.wptsAndBhvsList[self.xIndex][0]
            #print "lineType", self.xIndex, ":", self.lineType
            
            wptBhvPairs = self.extractWptBhvPairs()
            print "runOnce: wptBhvPairs: ", wptBhvPairs
            if rospy.is_shutdown():
                break
           
           # linesegment = ((x1,y1),(x2,y2))
           #if self.xIndex == 1
           #    Get ship's current postion
           #    Set first waypoint to ship's current position
           #    Set second waypoint to first point of mission
           # 
           
            self.setNextWaypoint(wptBhvPairs)
            
            self.xIndex += 1
            print "runOnce: self.xIndex: ", self.xIndex
        
        
        print "Mission complete."
        rospy.signal_shutdown("End of mission.")
     