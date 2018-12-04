#!/usr/bin/env python
"""
Created on Mon Nov 20 12:26:52 2017

"""

import rospy
import json
import mission_plan.missionplan
import project11_transformations
from geometry_msgs.msg import PoseStamped, Pose
from geographic_msgs.msg import GeoPointStamped
from geographic_msgs.msg import GeoPath
from geographic_msgs.msg import GeoPoseStamped
from std_msgs.msg import String, Float32, Int32, Bool
from mission_manager.msg import BehaviorControl
import math
from project11_transformations.srv import LatLongToMap
from project11_transformations.srv import LatLongToMapRequest
from dubins_curves.srv import DubinsCurves
from dubins_curves.srv import DubinsCurvesRequest
from marine_msgs.msg import NavEulerStamped
from tf.transformations import quaternion_from_euler
import project11

class MissionManager_Node():
    
    def __init__(self):
        rospy.init_node('MissionManager')
        
        self.waypointThreshold = 5.0
        self.currPosBoat_Map = None
        self.current_heading = None
        
        # Receive up-to-date map coordinates of ASV
        rospy.Subscriber('/position_map', PoseStamped, self.position_map_callback, queue_size = 1)
        # Receive up-to-date lat/lon coordinates of ASV
        rospy.Subscriber('/position', GeoPointStamped, self.position_callback, queue_size = 1)
        rospy.Subscriber('/heading', NavEulerStamped, self.heading_callback, queue_size = 1)
        # Receive up-to-date depth 
        rospy.Subscriber('/depth', Float32, self.depth_callback, queue_size = 1)
        # Receive up-to-date waypoint indices
        rospy.Subscriber('/moos/wpt_index', Int32, self.waypoint_index_callback, queue_size = 1)
        # Receive up-to-date waypoint updates
        rospy.Subscriber('/moos/wpt_updates', String, self.waypoint_updates_callback, queue_size = 1)
        rospy.Subscriber('/mission_plan', String, self.missionPlanCallback, queue_size = 1)
        
        # To publish waypoint updates to MOOS
        self.wpt_index_publisher = rospy.Publisher('/moos/wpt_index', Int32, queue_size = 10)
        self.wpt_updates_publisher = rospy.Publisher('/moos/wpt_updates', String, queue_size = 10)
        
        self.current_path_publisher = rospy.Publisher('/project11/current_path', GeoPath, queue_size = 10)
        
        self. update_timer = rospy.Timer(rospy.Duration.from_sec(0.1),self.update)
        
        self.mission = None
        self.nav_objectives = None
        self.current_nav_objective_index = None
        self.current_segment = None
   
    def position_map_callback(self, position_msg):
        self.currPosBoat_Map = position_msg
        
    def heading_callback(self, heading_msg):
        self.current_heading = heading_msg
    
    def position_callback(self, position_msg):
        # Save lat/lon coordinates
        self.currPosBoat_LatLon = position_msg        
        
    def depth_callback(self, depth_msg):
        # Save up-to-date depth
        self.depth = depth_msg
            
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
        
    def missionPlanCallback(self, mission_msg):
        print mission_msg
        self.mission = mission_plan.missionplan.Mission()
        self.mission.fromString(mission_msg.data)
        print self.mission
        self.parseMission()
    
    def parseMission(self):
        self.nav_objectives = []
        
        #rospy.wait_for_service('wgs84_to_map')
        #wgs84_to_map = rospy.ServiceProxy('wgs84_to_map', LatLongToMap)

        
        for nav_item in self.mission.plan['NAVIGATION']:
            #print nav_item
            try:
                if nav_item['type'] == 'survey_line':
                    self.nav_objectives.append(nav_item)
                    #for wp in self.nav_objectives[-1]['nav']:
                        #pos = wp['position']
                        #lltmr = LatLongToMapRequest()
                        #lltmr.wgs84.position.latitude = pos['latitude']
                        #lltmr.wgs84.position.longitude = pos['longitude']
                        #map_wp = wgs84_to_map(lltmr)
                        #pos['x'] = map_wp.map.point.x
                        #pos['y'] = map_wp.map.point.y
            except KeyError:
                pass
        print self.nav_objectives
        self.current_nav_objective_index = None

    def checkObjective(self):
        d = self.getDistanceToNextWaypoint()
        print d
        if d is not None and d < self.waypointThreshold:
            self.nextObjective()
    
    def nextObjective(self):
        if self.nav_objectives is not None and len(self.nav_objectives):
            last_current_wp = None
            if self.current_nav_objective_index is not None:
                last_current_wp = self.nav_objectives[self.current_nav_objective_index]['nav'][-1]
                self.current_nav_objective_index += 1
                if self.current_nav_objective_index >= len(self.nav_objectives):
                    self.current_nav_objective_index = 0
            else:
                self.current_nav_objective_index = 0
            print 'nav objective index:',self.current_nav_objective_index
            
            self.current_segment = self.nav_objectives[self.current_nav_objective_index]['nav'][0:2]
            print self.current_segment
            self.sendCurrentPathSegment(self.current_segment)
            #self.sendToMOOS(self.nav_objectives[self.current_nav_objective_index]['nav'],last_current_wp)
    
    def sendCurrentPathSegment(self, path_segment):
        gpath = GeoPath()
        gpath.header.stamp = rospy.Time.now()
        for s in path_segment:
            gpose = GeoPoseStamped()
            gpose.pose.position.latitude = s['position']['latitude']
            gpose.pose.position.longitude = s['position']['longitude']
            gpath.poses.append(gpose)
        self.current_path_publisher.publish(gpath)
        
    
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
        
    def getDistanceToNextWaypoint(self):
        if self.current_segment is not None:
            start_lat_rad = math.radians(self.current_segment[0]['position']['latitude'])
            start_lon_rad = math.radians(self.current_segment[0]['position']['longitude'])

            dest_lat_rad = math.radians(self.current_segment[1]['position']['latitude'])
            dest_lon_rad = math.radians(self.current_segment[1]['position']['longitude'])
            
            current_lat_rad = math.radians(self.currPosBoat_LatLon.position.latitude)
            current_lon_rad = math.radians(self.currPosBoat_LatLon.position.longitude)
            
            path_azimuth, path_distance = project11.geodesic.inverse(start_lon_rad, start_lat_rad, dest_lon_rad, dest_lat_rad)
            
            print 'path azimuth, distance:', path_azimuth, path_distance

            vehicle_azimuth, vehicle_distance = project11.geodesic.inverse(start_lon_rad, start_lat_rad, current_lon_rad, current_lat_rad)

            print 'vehicle azimuth, distance:', vehicle_azimuth, vehicle_distance
            
            error_azimuth = vehicle_azimuth - path_azimuth
            print 'error azimuth',error_azimuth
            sin_error_azimuth = math.sin(error_azimuth)
            cos_error_azimuth = math.cos(error_azimuth)
            
            cross_track = vehicle_distance*sin_error_azimuth
            progress = (vehicle_distance/path_distance)*cos_error_azimuth
            
            print 'cross track, progress',cross_track,progress
            
            azimuth,distance = project11.geodesic.inverse(current_lon_rad, current_lat_rad, dest_lon_rad, dest_lat_rad)
            return distance
        
        #Must subscribe to current position updates to determine
        #if self.currPosBoat_Map is not None and self.current_nav_objective_index is not None:
            #boat_x = self.currPosBoat_Map.pose.position.x
            #boat_y = self.currPosBoat_Map.pose.position.y
            
            #waypoint_x = self.last_wp['x']
            #waypoint_y = self.last_wp['y']
            
            #distance = math.sqrt((boat_x - waypoint_x)**2 + (boat_y - waypoint_y)**2)
            
            #print "distance: ", distance
            #return distance
        
    
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
    
    def sendToMOOS(self, navObjective, last_current_wp=None):
        print 'send to moos:'
        if len(navObjective) > 1:
            print navObjective[0]
            print navObjective[1]
        
        pointMsg = String()
        point = "points = "

        pairs = []
        
        #if last_current_wp is not None:
        #    pos = last_current_wp['position']
        #    pairs.append(str(pos['x'])+','+str(pos['y']))
        
        if self.current_heading is not None and self.currPosBoat_Map is not None and len(navObjective) > 1:
            # dubins to next line
            rospy.wait_for_service('dubins_curves')
            dubins_service = rospy.ServiceProxy('dubins_curves', DubinsCurves)
            
            dubins_req = DubinsCurvesRequest()
            dubins_req.radius = 10.0
            dubins_req.samplingInterval = 2.0
            dubins_req.startPose = self.currPosBoat_Map.pose
            dubins_req.targetPose.position.x = navObjective[0]['position']['x']
            dubins_req.targetPose.position.y = navObjective[0]['position']['y']
            
            target_yaw = math.radians(self.headToYaw(self.current_heading.orientation.heading))
            if len(navObjective) > 1:
                dx = navObjective[1]['position']['x'] - navObjective[0]['position']['x']
                dy = navObjective[1]['position']['y'] - navObjective[0]['position']['y']
                target_yaw = math.atan2(dy,dx)
            
            q = quaternion_from_euler(0.0,0.0,target_yaw)
            dubins_req.targetPose.orientation.x = q[0]
            dubins_req.targetPose.orientation.y = q[1]
            dubins_req.targetPose.orientation.z = q[2]
            dubins_req.targetPose.orientation.w = q[3]
            print dubins_req
            dubins_path = dubins_service(dubins_req)
            print dubins_path
            for p in dubins_path.path.poses:
                pairs.append(str(p.position.x)+','+str(p.position.y))
            
        for wp in navObjective:
            print wp
            pos = wp['position']
            print pos
            self.last_wp = pos
            pairs.append(str(pos['x'])+','+str(pos['y']))

        point += " : ".join(pairs)
        
        pointMsg.data = point
        
        print "POINT:", pointMsg
        
        # Publish path/waypoint(s) to MOOS
        self.wpt_updates_publisher.publish(pointMsg)

    def setNextWaypoint(self, wptBhvPairs):
        print "In setNextWaypoint."
        # NOTE: wptBhvPairs (when received from extractWptBhvPairs function) is organized as follows:
        # [[[wpt_x, wpt_y], [wpt_bhv, wpt_bhv, ...]], [[wpt_x, wpt_y], [wpt_bhv, wpt_bhv, ...]]...]
        
        self.sendToMOOS(wptBhvPairs)

    def update(self, event):
        self.checkObjective()
        if self.nav_objectives is not None and self.current_nav_objective_index is None:
            self.nextObjective()
     
    def run(self):
        rospy.spin()

