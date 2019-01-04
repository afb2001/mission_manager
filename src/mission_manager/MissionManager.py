#!/usr/bin/env python
"""
Created on Mon Nov 20 12:26:52 2017

"""

import rospy
import json
import math
import mission_plan.missionplan
import project11_transformations
import project11
from tf.transformations import quaternion_from_euler

from dubins_curves.srv import DubinsCurvesLatLong
from dubins_curves.srv import DubinsCurvesLatLongRequest
from geographic_msgs.msg import GeoPointStamped
from geographic_msgs.msg import GeoPath
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Pose
from marine_msgs.msg import Heartbeat
from marine_msgs.msg import KeyValue
from marine_msgs.msg import NavEulerStamped
from mission_manager.msg import BehaviorControl
from project11_transformations.srv import LatLongToMap
from project11_transformations.srv import LatLongToMapRequest
from std_msgs.msg import String, Float32, Int32, Bool

from dynamic_reconfigure.server import Server
from mission_manager.cfg import mission_managerConfig

class MissionManager_Node():
    
    def __init__(self):
        rospy.init_node('MissionManager')
        
        self.waypointThreshold = 10.0
        self.turnRadius = 20.0
        self.segmentLength = 15.0
        
        self.position = None
        self.heading = None
        
        rospy.Subscriber('/position', GeoPointStamped, self.position_callback, queue_size = 1)
        rospy.Subscriber('/heading', NavEulerStamped, self.heading_callback, queue_size = 1)
        rospy.Subscriber('/depth', Float32, self.depth_callback, queue_size = 1)
        rospy.Subscriber('/mission_plan', String, self.missionPlanCallback, queue_size = 1)
        rospy.Subscriber('/helm_mode', String, self.helmModeCallback, queue_size = 1)
        
        self.current_path_publisher = rospy.Publisher('/project11/mission_manager/current_path', GeoPath, queue_size = 10)
        self.current_speed_publisher = rospy.Publisher('/project11/mission_manager/current_speed', Float32, queue_size = 10)
        self.crosstrack_error_publisher = rospy.Publisher('/project11/mission_manager/crosstrack_error', Float32, queue_size = 10)
        self.path_progress_publisher = rospy.Publisher('/project11/mission_manager/path_progress', Float32, queue_size = 10)
        self.distance_to_waypoint_publisher  = rospy.Publisher('/project11/mission_manager/distance_to_waypoint', Float32, queue_size = 10)
        self.status_publisher = rospy.Publisher('/project11/mission_manager/status', Heartbeat, queue_size = 10)
        
        self.update_timer = rospy.Timer(rospy.Duration.from_sec(0.1),self.update)
        
        self.config_server = Server(mission_managerConfig, self.reconfigure_callback)
        
        self.mission = None
        self.nav_objectives = None
        self.default_speed = None
        self.current_nav_objective_index = None
        self.current_segment = None
        self.state = 'idle'
        self.helm_mode = 'unknown'
   
    def reconfigure_callback(self, config, level):
        self.waypointThreshold = config['waypoint_threshold']
        self.turnRadius = config['turn_radius']
        self.segmentLength = config['segment_length']
        return config

    def heading_callback(self, heading_msg):
        self.heading = heading_msg
    
    def position_callback(self, position_msg):
        self.position = position_msg        
        
    def depth_callback(self, depth_msg):
        self.depth = depth_msg
        
    def helmModeCallback(self, msg):
        self.helm_mode = msg.data
            
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
        
        if 'defaultspeed_ms' in self.mission.plan['DEFAULT_PARAMETERS']:
            self.default_speed = self.mission.plan['DEFAULT_PARAMETERS']['defaultspeed_ms']
        
        for nav_item in self.mission.plan['NAVIGATION']:
            #print nav_item
            try:
                if nav_item['type'] == 'survey_line':
                    self.nav_objectives.append(nav_item)
            except KeyError:
                pass
        print self.nav_objectives
        self.current_nav_objective_index = None
        if len(self.nav_objectives):
            #self.current_nav_objective_index = 0
            self.state = 'pre-mission'
        else:
            self.state = 'idle'

    def checkObjective(self):
        if self.helm_mode == 'survey':
            if self.state == 'transit':
                d = self.getDistanceToNextWaypoint()
                #print d
                if d is not None and d['distance_to_waypoint'] < self.waypointThreshold or (1-d['progress'])*d['path_distance'] < self.waypointThreshold:
                    self.current_transit_path_index += 1
                    if self.current_transit_path_index >= len(self.current_transit_path)-1:
                        self.current_segment = ((self.nav_objectives[self.current_nav_objective_index]['nav'][0]['position']['latitude'],
                                                self.nav_objectives[self.current_nav_objective_index]['nav'][0]['position']['longitude']),
                                                (self.nav_objectives[self.current_nav_objective_index]['nav'][1]['position']['latitude'],
                                                self.nav_objectives[self.current_nav_objective_index]['nav'][1]['position']['longitude']))
                        self.sendCurrentPathSegment(self.current_segment)
                        self.state = 'line-following'
                    else:
                        self.current_segment = []
                        for p in self.current_transit_path[self.current_transit_path_index:]:
                            self.current_segment.append((p.position.latitude,p.position.longitude))

                        #self.current_segment = ((self.current_transit_path[self.current_transit_path_index].position.latitude,
                        #                         self.current_transit_path[self.current_transit_path_index].position.longitude),
                        #                        (self.current_transit_path[self.current_transit_path_index+1].position.latitude,
                        #                         self.current_transit_path[self.current_transit_path_index+1].position.longitude))
                        self.sendCurrentPathSegment(self.current_segment)

            if self.state == 'line-following':
                d = self.getDistanceToNextWaypoint()
                #print d
                if d is not None and d['distance_to_waypoint'] < self.waypointThreshold or (1-d['progress'])*d['path_distance'] < self.waypointThreshold:
                    #self.nextObjective()
                    if self.current_segment_index >= len(self.nav_objectives[self.current_nav_objective_index]['nav'])-2:
                        self.state = 'line-end'
                    else:
                        self.current_segment_index += 1
                        speed = None
                        if 'parameters' in self.nav_objectives[self.current_nav_objective_index] and 'speed_ms' in self.nav_objectives[self.current_nav_objective_index]['parameters']:
                            speed = self.nav_objectives[self.current_nav_objective_index]['parameters']['speed_ms']
                        self.current_segment = ((self.nav_objectives[self.current_nav_objective_index]['nav'][self.current_segment_index]['position']['latitude'],
                                                self.nav_objectives[self.current_nav_objective_index]['nav'][self.current_segment_index]['position']['longitude']),
                                                (self.nav_objectives[self.current_nav_objective_index]['nav'][self.current_segment_index+1]['position']['latitude'],
                                                self.nav_objectives[self.current_nav_objective_index]['nav'][self.current_segment_index+1]['position']['longitude']))
                        self.sendCurrentPathSegment(self.current_segment, speed)
                        
            if self.state == 'pre-mission' or self.state == 'line-end':
                if self.position is not None:
                    self.nextObjective()
                    start_point = self.nav_objectives[self.current_nav_objective_index]['nav'][0]
                    next_point = self.nav_objectives[self.current_nav_objective_index]['nav'][1]
                    print start_point
                    print self.position
                    if self.distanceTo(start_point['position']['latitude'],start_point['position']['longitude']) > self.waypointThreshold:
                        self.state = 'transit'
                        self.current_transit_path = self.generatePath(self.position.position.latitude,self.position.position.longitude,self.heading.orientation.heading,
                                        start_point['position']['latitude'],start_point['position']['longitude'],self.segmentHeading(
                                            start_point['position']['latitude'],start_point['position']['longitude'],next_point['position']['latitude'],next_point['position']['longitude']))
                        self.current_transit_path_index = 0
                        if len(self.current_transit_path)>=2:
                            self.current_segment = []
                            for p in self.current_transit_path:
                                self.current_segment.append((p.position.latitude,p.position.longitude))
                            #self.current_segment = ((self.current_transit_path[0].position.latitude, self.current_transit_path[0].position.longitude),
                            #                        (self.current_transit_path[1].position.latitude, self.current_transit_path[1].position.longitude))
                            self.sendCurrentPathSegment(self.current_segment)
                    else:
                        self.state = 'line-following'
            
    
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
            self.current_segment_index = 0
            print 'nav objective index:',self.current_nav_objective_index
            
            #self.current_segment = self.nav_objectives[self.current_nav_objective_index]['nav'][0:2]
            #print self.current_segment
            #self.sendCurrentPathSegment(self.current_segment)
            #self.sendToMOOS(self.nav_objectives[self.current_nav_objective_index]['nav'],last_current_wp)
    
    def sendCurrentPathSegment(self, path_segment, speed=None):
        gpath = GeoPath()
        gpath.header.stamp = rospy.Time.now()
        for s in path_segment:
            gpose = GeoPoseStamped()
            gpose.pose.position.latitude = s[0]
            gpose.pose.position.longitude = s[1]
            gpath.poses.append(gpose)
        self.current_path_publisher.publish(gpath)
        if speed is not None:
            self.current_speed_publisher.publish(speed)
        else:
            if self.default_speed is not None:
                self.current_speed_publisher.publish(self.default_speed)

    def generatePath(self, startLat, startLon, startHeading, targetLat, targetLon, targetHeading):
        rospy.wait_for_service('dubins_curves_latlong')
        dubins_service = rospy.ServiceProxy('dubins_curves_latlong', DubinsCurvesLatLong)

        dubins_req = DubinsCurvesLatLongRequest()
        dubins_req.radius = self.turnRadius
        dubins_req.samplingInterval = self.segmentLength

        dubins_req.startGeoPose.position.latitude = startLat
        dubins_req.startGeoPose.position.longitude = startLon

        start_yaw = math.radians(self.headToYaw(startHeading))
        start_quat = quaternion_from_euler(0.0,0.0,start_yaw)
        dubins_req.startGeoPose.orientation.x = start_quat[0]
        dubins_req.startGeoPose.orientation.y = start_quat[1]
        dubins_req.startGeoPose.orientation.z = start_quat[2]
        dubins_req.startGeoPose.orientation.w = start_quat[3]
        
        dubins_req.targetGeoPose.position.latitude = targetLat
        dubins_req.targetGeoPose.position.longitude = targetLon
      
        target_yaw = math.radians(self.headToYaw(targetHeading))
        q = quaternion_from_euler(0.0,0.0,target_yaw)
        dubins_req.targetGeoPose.orientation.x = q[0]
        dubins_req.targetGeoPose.orientation.y = q[1]
        dubins_req.targetGeoPose.orientation.z = q[2]
        dubins_req.targetGeoPose.orientation.w = q[3]

        print dubins_req
        dubins_path = dubins_service(dubins_req)
        print dubins_path
        return dubins_path.path
        
    
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

    def distanceTo(self, lat, lon):
        current_lat_rad = math.radians(self.position.position.latitude)
        current_lon_rad = math.radians(self.position.position.longitude)
        target_lat_rad = math.radians(lat)
        target_lon_rad = math.radians(lon)
        azimuth, distance = project11.geodesic.inverse(current_lon_rad, current_lat_rad, target_lon_rad, target_lat_rad)
        return distance

    def segmentHeading(self,lat1,lon1,lat2,lon2):
        start_lat_rad = math.radians(lat1)
        start_lon_rad = math.radians(lon1)

        dest_lat_rad = math.radians(lat2)
        dest_lon_rad = math.radians(lon2)
        
        path_azimuth, path_distance = project11.geodesic.inverse(start_lon_rad, start_lat_rad, dest_lon_rad, dest_lat_rad)
        return math.degrees(path_azimuth)
        
        
    def getDistanceToNextWaypoint(self):
        if self.current_segment is not None:
            ret = {}
            start_lat_rad = math.radians(self.current_segment[0][0])
            start_lon_rad = math.radians(self.current_segment[0][1])

            dest_lat_rad = math.radians(self.current_segment[1][0])
            dest_lon_rad = math.radians(self.current_segment[1][1])
            
            current_lat_rad = math.radians(self.position.position.latitude)
            current_lon_rad = math.radians(self.position.position.longitude)
            
            path_azimuth, path_distance = project11.geodesic.inverse(start_lon_rad, start_lat_rad, dest_lon_rad, dest_lat_rad)
            ret['path_azimuth'] = path_azimuth
            ret['path_distance'] = path_distance
            
            #print 'path azimuth, distance:', path_azimuth, path_distance

            vehicle_azimuth, vehicle_distance = project11.geodesic.inverse(start_lon_rad, start_lat_rad, current_lon_rad, current_lat_rad)
            ret['vehicle_azimuth'] = vehicle_azimuth
            ret['vehicle_distance'] = vehicle_distance

            #print 'vehicle azimuth, distance:', vehicle_azimuth, vehicle_distance
            
            error_azimuth = vehicle_azimuth - path_azimuth
            #print 'error azimuth',error_azimuth
            sin_error_azimuth = math.sin(error_azimuth)
            cos_error_azimuth = math.cos(error_azimuth)
            
            cross_track = vehicle_distance*sin_error_azimuth
            progress = (vehicle_distance/path_distance)*cos_error_azimuth
            ret['cross_track'] = cross_track
            ret['progress'] = progress
            self.crosstrack_error_publisher.publish(cross_track)
            self.path_progress_publisher.publish(progress)
            
            #print 'cross track, progress',cross_track,progress
            
            azimuth,distance = project11.geodesic.inverse(current_lon_rad, current_lat_rad, dest_lon_rad, dest_lat_rad)
            self.distance_to_waypoint_publisher.publish(distance)
            ret['azimuth_to_waypoint'] = azimuth
            ret['distance_to_waypoint'] = distance
            return ret
        

    def update(self, event):
        self.checkObjective()
        if self.nav_objectives is not None and self.current_nav_objective_index is None:
            self.nextObjective()
        hb = Heartbeat()
        hb.header.stamp = rospy.Time.now()
        kv = KeyValue()
        kv.key = 'state'
        kv.value = self.state
        hb.values.append(kv)
        self.status_publisher.publish(hb)
     
    def run(self):
        rospy.spin()

