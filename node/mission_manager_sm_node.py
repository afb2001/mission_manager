#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String
from geographic_msgs.msg import GeoPointStamped
from marine_msgs.msg import NavEulerStamped
from geographic_msgs.msg import GeoPoseStamped
from geographic_msgs.msg import GeoPose

from dynamic_reconfigure.server import Server
from mission_manager.cfg import mission_managerConfig

from dubins_curves.srv import DubinsCurvesLatLong
from dubins_curves.srv import DubinsCurvesLatLongRequest

import actionlib
import path_follower.msg
import hover.msg

import project11
from tf.transformations import quaternion_from_euler

import json
import math

class MissionManagerCore:
    def __init__(self):
        self.piloting_mode = 'standby'
        self.position = None
        self.heading = None

        self.tasks = []
        self.current_task_index = None
        self.override = None # hover or goto that temporarily overrides current task
        
        rospy.Subscriber('/project11/piloting_mode', String, self.pilotingModeCallback, queue_size = 1)
        rospy.Subscriber('/position', GeoPointStamped, self.positionCallback, queue_size = 1)
        rospy.Subscriber('/heading', NavEulerStamped, self.headingCallback, queue_size = 1)
        rospy.Subscriber('/project11/mission_manager/command', String, self.commandCallback, queue_size = 1)

        self.config_server = Server(mission_managerConfig, self.reconfigure_callback)

    def pilotingModeCallback(self, msg):
        self.piloting_mode = msg.data

    def positionCallback(self, msg):
        self.position = msg
        
    def headingCallback(self, msg):
        self.heading = msg

    def reconfigure_callback(self, config, level):
        self.waypointThreshold = config['waypoint_threshold']
        self.turnRadius = config['turn_radius']
        self.segmentLength = config['segment_length']
        self.default_speed = config['default_speed']
        return config
        
    def getPilotingMode(self):
        return self.piloting_mode
    
    def hasPendingTasks(self):
        if len(self.tasks):
            return True
        return False

    def commandCallback(self, msg):
        parts = msg.data.split(None,1)
        cmd = parts[0]
        if len(parts) > 1:
            args = parts[1]
        else:
            args = None
                
        #print 'command:',cmd,'args:',args
        
        if cmd == 'replace_task':
            self.clearTasks()
            self.addTask(args)
            
        if cmd == 'append_task':
            self.addTask(args)

        if cmd == 'prepend_task':
            self.addTask(args, True)
            
        if cmd == 'clear_tasks':
            self.clearTasks()
            
        if cmd == 'next_task':
            pass
        
        if cmd == 'prev_task':
            pass
        
        if cmd == 'goto_task':
            pass

        if cmd == 'goto_line':
            pass
        
        if cmd == 'restart_line':
            pass
        
        if cmd == 'override':
            parts = args.split(None,1)
            if len(parts) == 2:
                task_type = parts[0]   
                task = None
                if task_type == 'goto':
                    task = self.parseLatLong(args)
                    if task is not None:
                        task['type'] = 'goto'
                        task['override'] = True
                if task_type == 'hover':
                    task = self.parseLatLong(args)
                    if task is not None:
                        task['type'] = 'hover'
                        task['override'] = True
                if task is not None:
                    self.setOverride(task)

    def clearTasks(self):
        self.tasks = []
        self.current_task_index = None

    def addTask(self, args, prepend=False):
        parts = args.split(None,1)
        if len(parts) == 2:
            task_type = parts[0]
            task = None
            if task_type == 'mission_plan':
                task = self.parseMission(parts[1])
            if task_type == 'goto':
                task = self.parseLatLong(args)
                if task is not None:
                    task['type'] = 'goto'
            if task_type == 'hover':
                task = self.parseLatLong(args)
                if task is not None:
                    task['type'] = 'hover'
            if task is not None: 
                if prepend:
                    self.tasks.insert(0,task)
                    if self.current_task_index is not None:
                        self.current_task_index += 1
                else:
                    self.tasks.append(task)
                
    def parseLatLong(self,args):
        latlon = args.split()
        if len(latlon) == 2:
            try:
                lat = float(lat)
                lon = float(lon)
                return {'latitude':lat, 'longitude':lon}
            except ValueError:
                return None
        
        
    def parseMission(self, mp):
        ret = {'type':'mission_plan',
               'nav_objectives':[],
               'default_speed':self.default_speed
               }
        
        plan = json.loads(mp)
        
        if 'defaultspeed_ms' in plan['DEFAULT_PARAMETERS']:
            ret['default_speed'] = plan['DEFAULT_PARAMETERS']['defaultspeed_ms']

        for nav_item in plan['NAVIGATION']:
            try:
                if nav_item['type'] == 'survey_line':
                    ret['nav_objectives'].append(nav_item)
            except KeyError:
                pass
        ret['current_nav_objective_index'] = None
        return ret

    def distanceTo(self, lat, lon):
        current_lat_rad = math.radians(self.position.position.latitude)
        current_lon_rad = math.radians(self.position.position.longitude)
        target_lat_rad = math.radians(lat)
        target_lon_rad = math.radians(lon)
        azimuth, distance = project11.geodesic.inverse(current_lon_rad, current_lat_rad, target_lon_rad, target_lat_rad)
        return distance

    def generatePathFromVehicle(self, targetLat, targetLon, targetHeading):
        return self.generatePath(self.position.position.latitude, self.position.position.longitude, self.heading.orientation.heading, targetLat, targetLon, targetHeading)

    def generatePath(self, startLat, startLon, startHeading, targetLat, targetLon, targetHeading):
        rospy.wait_for_service('dubins_curves_latlong')
        dubins_service = rospy.ServiceProxy('dubins_curves_latlong', DubinsCurvesLatLong)

        dubins_req = DubinsCurvesLatLongRequest()
        dubins_req.radius = self.turnRadius
        dubins_req.samplingInterval = self.segmentLength

        dubins_req.startGeoPose.position.latitude = startLat
        dubins_req.startGeoPose.position.longitude = startLon

        start_yaw = math.radians(self.headingToYaw(startHeading))
        start_quat = quaternion_from_euler(0.0,0.0,start_yaw)
        dubins_req.startGeoPose.orientation.x = start_quat[0]
        dubins_req.startGeoPose.orientation.y = start_quat[1]
        dubins_req.startGeoPose.orientation.z = start_quat[2]
        dubins_req.startGeoPose.orientation.w = start_quat[3]
        
        dubins_req.targetGeoPose.position.latitude = targetLat
        dubins_req.targetGeoPose.position.longitude = targetLon
      
        target_yaw = math.radians(self.headingToYaw(targetHeading))
        q = quaternion_from_euler(0.0,0.0,target_yaw)
        dubins_req.targetGeoPose.orientation.x = q[0]
        dubins_req.targetGeoPose.orientation.y = q[1]
        dubins_req.targetGeoPose.orientation.z = q[2]
        dubins_req.targetGeoPose.orientation.w = q[3]

        #print dubins_req
        dubins_path = dubins_service(dubins_req)
        #print dubins_path
        return dubins_path.path

    def segmentHeading(self,lat1,lon1,lat2,lon2):
        start_lat_rad = math.radians(lat1)
        start_lon_rad = math.radians(lon1)

        dest_lat_rad = math.radians(lat2)
        dest_lon_rad = math.radians(lon2)
        
        path_azimuth, path_distance = project11.geodesic.inverse(start_lon_rad, start_lat_rad, dest_lon_rad, dest_lat_rad)
        return math.degrees(path_azimuth)

    def  headingToPoint(self,lat,lon):
        return self.segmentHeading(self.position.position.latitude, self.position.position.longitude, lat, lon)
    
    def headingToYaw(self, heading):
        return 90-heading
               
class MMState(smach.State):
    '''
    Base state for Mission Manager states
    '''
    def __init__(self, mm, outcomes):
        smach.State.__init__(self, outcomes=outcomes)
        self.missionManager = mm
        
class Pause(MMState):
    """
    This state is for all top level piloting_mode other than autonomous
    """
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['resume','exit'])
        
    def execute(self, userdata):
        while self.missionManager.getPilotingMode() != 'autonomous':
            if rospy.is_shutdown():
                return 'exit'
            rospy.sleep(0.1)
        return 'resume'

class Idle(MMState):
    """
    In autonomous mode, but with no pending tasks.
    """
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['exit','do-task','pause'])
        
    def execute(self, userdata):
        while not self.missionManager.hasPendingTasks():
            if rospy.is_shutdown():
                return 'exit'
            if self.missionManager.getPilotingMode() != 'autonomous':
                return 'pause'
            rospy.sleep(0.1)
        return 'do-task'

class Resume(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['idle','goto','hover','follow_path'])
                         
    def execute(self, userdate):
        if self.missionManager.current_task is not None:
            pass
        return 'idle'

class NextTask(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['idle','mission_plan','goto','hover'])
        
    def execute(self, userdata):
        if self.missionManager.override is not None:
            if self.missionManager.override['type'] in('goto', 'hover'):
                self.missionManager.current_task = self.missionManager.override
                return self.missionManager.override['type']
        if self.missionManager.current_task_index is None:
            self.missionManager.current_task_index = 0
        else:
            if self.missionManager.current_task['type'] == 'mission_plan':
                if self.missionManager.current_task['current_nav_objective_index'] >= len(self.missionManager.current_task['nav_objectives']):
                    self.missionManager.current_task_index += 1
            else:
                self.missionManager.current_task_index += 1
        if self.missionManager.current_task_index < len(self.missionManager.tasks):
            self.missionManager.current_task = self.missionManager.tasks[self.missionManager.current_task_index]
            return self.missionManager.current_task['type']
        else:
            self.missionManager.current_task_index = None
        return 'idle'

class Hover(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['cancelled','exit','pause'])
        self.hover_client = actionlib.SimpleActionClient('hover_action', hover.msg.hoverAction)
        self.task = None
        
    def execute(self, userdata):
        if self.missionManager.current_task is not None:
            self.task = self.missionManager.current_task
            goal = hover.msg.hoverGoal()
            goal.target.latitude = self.task['latitude']
            goal.target.longitude = self.task['longitude']
            self.hover_client.wait_for_server()
            self.hover_client.send_goal(goal)
        while self.missionManager.current_task == self.task:
            if rospy.is_shutdown():
                return 'exit'
            if self.missionManager.getPilotingMode() != 'autonomous':
                return 'pause'
        self.hover_client.cancel_goal()
        self.task = None
        return 'cancelled'

class MissionPlan(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['follow_path','done'])
        self.task = None
        
    def execute(self, userdata):
        # are we resuming?
        if self.task == self.missionManager.current_task:
            if self.task['current_nav_objective_index'] >= len(self.task['nav_objectives']):
                self.task['current_nav_objective_index'] = None
                self.task = None
                return 'done'
            if not 'current_path' in self.task or self.task['current_path'] is None:
                self.generatePaths()
            return 'follow_path'
        else:
            self.task = self.missionManager.current_task
            self.task['current_nav_objective_index'] = 0
            self.generatePaths()
            return 'follow_path'
        self.task = None
        return 'done'

    def generatePaths(self):
        path = []
        for p in self.task['nav_objectives'][self.task['current_nav_objective_index']]['nav']:
            #path.append((p['position']['latitude'],p['position']['longitude']))
            gp = GeoPose();
            gp.position.latitude = p['position']['latitude']
            gp.position.longitude = p['position']['longitude']
            path.append(gp)
        self.task['current_path'] = path
        # decide if we transit or start line
        self.task['transit_path'] = None
        if len(self.task['current_path']) >1:
            start_point = self.task['current_path'][0]
            next_point = self.task['current_path'][1]
            if self.missionManager.distanceTo(start_point.position.latitude,start_point.position.longitude) > self.missionManager.waypointThreshold:
                #transit
                transit_path = self.missionManager.generatePathFromVehicle(start_point.position.latitude,start_point.position.longitude, self.missionManager.segmentHeading(start_point.position.latitude,start_point.position.longitude,next_point.position.latitude,next_point.position.longitude))
                self.task['transit_path'] = transit_path
        
class Goto(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['done','follow_path'])
        self.task = None
        
    def execute(self, userdata):
        if self.missionManager.distanceTo(task['latitude'],task['longitude']) <= self.missionManager.waypointThreshold:
            return 'done'
        # are we resuming?
        if self.task == self.missionManager.current_task:
            return 'follow_path'
        else:
            self.task = self.missionManager.current_task
            headingToPoint = self.missionManager.headingToPoint(self.task['latitude'],self.task['longitude'])
            path = self.generatePathFromVehicle(self.task['latitude'],self.task['longitude'],headingToPoint)
            self.task['path'] = path
            return 'follow_path'
        
class FollowPath(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['done','cancelled','exit','pause'])
        self.path_follower_client = actionlib.SimpleActionClient('path_follower_action', path_follower.msg.path_followerAction)
        self.task = None
        self.task_complete = False
        
    def execute(self, userdata):
        if self.missionManager.current_task is not None:
            self.task = self.missionManager.current_task
            goal = path_follower.msg.path_followerGoal()
            goal.path.header.stamp = rospy.Time.now()
            if self.task['type'] == 'goto':
                path = self.task['path']
            if self.task['type'] == 'mission_plan':
                if self.task['transit_path'] is not None:
                    path = self.task['transit_path']
                else:
                    path = self.task['current_path']
            for s in path:
                #print s
                gpose = GeoPoseStamped()
                gpose.pose = s
                goal.path.poses.append(gpose)
            goal.speed = self.task['default_speed']
            self.task_complete = False
            self.path_follower_client.wait_for_server()
            self.path_follower_client.send_goal(goal, self.path_follower_done_callback, self.path_follower_active_callback, self.path_follower_feedback_callback)

        while self.missionManager.current_task == self.task:
            if rospy.is_shutdown():
                return 'exit'
            if self.missionManager.getPilotingMode() != 'autonomous':
                return 'pause'
            if self.task_complete:
                if self.task['type'] == 'mission_plan':
                    if self.task['transit_path'] is not None:
                        self.task['transit_path'] = None
                    else:
                        self.task['current_path'] = None
                        self.task['current_nav_objective_index'] += 1
                self.task = None
                return 'done'
        self.path_follower_client.cancel_goal()
        self.task = None
        return 'cancelled'


    def path_follower_done_callback(self, status, result):
        self.task_complete = True
    
    def path_follower_active_callback(self):
        pass
    
    def path_follower_feedback_callback(self, msg):
        pass
    
def main():
    rospy.init_node('MissionManager')
    
    missionManager = MissionManagerCore()

    sm_top = smach.StateMachine(outcomes=['exit'])
    
    with sm_top:
        smach.StateMachine.add('PAUSE', Pause(missionManager), transitions={'resume':'AUTONOMOUS', 'exit':'exit'})
        
        sm_auto = smach.StateMachine(outcomes=['pause','exit'])
        
        with sm_auto:
            smach.StateMachine.add('IDLE', Idle(missionManager), transitions={'do-task':'NEXTTASK', 'pause':'pause'})
            smach.StateMachine.add('NEXTTASK', NextTask(missionManager), transitions={'idle':'IDLE', 'mission_plan':'MISSIONPLAN', 'hover':'HOVER', 'goto':'GOTO'})
            smach.StateMachine.add('HOVER', Hover(missionManager), transitions={'pause':'pause', 'cancelled':'NEXTTASK'})
            smach.StateMachine.add('MISSIONPLAN', MissionPlan(missionManager), transitions={'done':'NEXTTASK', 'follow_path':'FOLLOWPATH'})
            smach.StateMachine.add('GOTO',Goto(missionManager), transitions={'done':'NEXTTASK', 'follow_path':'FOLLOWPATH'})
            smach.StateMachine.add('FOLLOWPATH', FollowPath(missionManager), transitions={'pause':'pause', 'cancelled':'NEXTTASK', 'done':'NEXTTASK'})

        smach.StateMachine.add('AUTONOMOUS', sm_auto, transitions={'pause':'PAUSE', 'exit':'exit'})
    
    sis = smach_ros.IntrospectionServer('mission_manager', sm_top, '/mission_manager')
    sis.start()                                                                            

    sm_top.execute()
    rospy.spin()
    
if __name__ == '__main__':
    main()
