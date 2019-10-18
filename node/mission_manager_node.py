#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import String
from geographic_msgs.msg import GeoPointStamped
from marine_msgs.msg import NavEulerStamped
from marine_msgs.msg import Heartbeat
from marine_msgs.msg import KeyValue
from geographic_msgs.msg import GeoPoseStamped
from geographic_msgs.msg import GeoPose

from dynamic_reconfigure.server import Server
from mission_manager.cfg import mission_managerConfig

from dubins_curves.srv import DubinsCurvesLatLong
from dubins_curves.srv import DubinsCurvesLatLongRequest

import actionlib
import path_follower.msg
import path_planner.msg
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

        self.tasks = [] # list of tasks to do, or already done. Keeping all the tasks allows us to run them in a loop
        self.pending_tasks = [] # list of tasks to be done. Once a task is completed, it is dropped from this list. Overrides get prepended here.
        self.current_task = None
        
        rospy.Subscriber('/project11/piloting_mode', String, self.pilotingModeCallback, queue_size = 1)
        rospy.Subscriber('/position', GeoPointStamped, self.positionCallback, queue_size = 1)
        rospy.Subscriber('/heading', NavEulerStamped, self.headingCallback, queue_size = 1)
        rospy.Subscriber('/project11/mission_manager/command', String, self.commandCallback, queue_size = 1)
        
        self.status_publisher = rospy.Publisher('/project11/mission_manager/status', Heartbeat, queue_size = 10)

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
        if config['planner'] == 0:
            self.planner = 'path_follower'
        elif config['planner'] == 1:
            self.planner = 'path_planner'
        return config
        
    def getPilotingMode(self):
        return self.piloting_mode
    
    def hasPendingTasks(self):
        if len(self.pending_tasks):
            return True
        if len(self.tasks):
            self.pending_tasks = list(self.tasks)
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
            self.current_task = None
        
        if cmd == 'prev_task':
            pass
        
        if cmd == 'goto_task':
            pass

        if cmd == 'goto_line':
            task = self.current_task
            if task is not None and task['type'] == 'mission_plan':
                task['current_nav_objective_index'] = int(args)
                task['do_transit'] = False
                task['current_path'] = None
                self.pending_tasks = [task]+self.pending_tasks
                self.current_task = None
        
        if cmd == 'start_line':
            task = self.current_task
            if task is not None and task['type'] == 'mission_plan':
                task['current_nav_objective_index'] = int(args)
                task['do_transit'] = True
                task['current_path'] = None
                self.pending_tasks = [task]+self.pending_tasks
                self.current_task = None
        
        if cmd == 'override':
            parts = args.split(None,1)
            if len(parts) == 2:
                task_type = parts[0]   
                task = None
                if task_type == 'goto':
                    task = self.parseLatLong(parts[1])
                    if task is not None:
                        task['type'] = 'goto'
                        task['override'] = True
                if task_type == 'hover':
                    task = self.parseLatLong(parts[1])
                    if task is not None:
                        task['type'] = 'hover'
                        task['override'] = True
                if task is not None:
                    self.setOverride(task)

    def clearTasks(self):
        self.tasks = []
        self.current_task = None

    def addTask(self, args, prepend=False):
        parts = args.split(None,1)
        print parts
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
                    if self.current_task is not None:
                        self.pending_tasks.insert(0,self.pending_tasks)
                    self.pending_tasks.insert(0,task)
                else:
                    self.tasks.append(task)
                    self.pending_tasks.append(task)
        print 'tasks'
        print self.tasks
        print 'pending tasks'
        print self.pending_tasks

    def setOverride(self, task):
        if self.current_task is not None:
            if not 'override' in self.current_task or not self.current_task['override']:
                self.pending_tasks.insert(0,self.current_task)
        self.pending_tasks.insert(0,task)
        task['override'] = True
        self.current_task = None
        
    def parseLatLong(self,args):
        latlon = args.split()
        if len(latlon) >= 2:
            try:
                lat = float(latlon[0])
                lon = float(latlon[1])
                return {'latitude':lat, 'longitude':lon}
            except ValueError:
                return None
        
    def parseMission(self, mp):
        ret = {'type':'mission_plan',
               'nav_objectives':[],
               'default_speed':self.default_speed,
               'do_transit':True
               }
        
        plan = json.loads(mp)
        
        for item in plan:
            #print item
            if item['type'] == 'Platform':
                ret['default_speed'] = item['speed']*0.514444  # knots to m/s
            if item['type'] == 'SurveyPattern':
                for c in item['children']:
                    ret['nav_objectives'].append(c)
                ret['label'] = item['label']
            if item['type'] == 'TrackLine':
                ret['nav_objectives'].append(item)
                ret['label'] = item['label']
            
        
        ret['current_nav_objective_index'] = 0
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

    def headingToPoint(self,lat,lon):
        return self.segmentHeading(self.position.position.latitude, self.position.position.longitude, lat, lon)
    
    def headingToYaw(self, heading):
        return 90-heading
    
    def publishStatus(self, state):
        hb = Heartbeat()
        hb.header.stamp = rospy.Time.now()

        hb.values.append(KeyValue('state',state))
        hb.values.append(KeyValue('tasks_count',str(len(self.tasks))))
        for t in self.tasks:
            tstring = t['type']
            if t['type'] == 'mission_plan':
                tstring += ' ('+t['label']+')'
            hb.values.append(KeyValue('-task',tstring))

        hb.values.append(KeyValue('pending_tasks_count',str(len(self.pending_tasks))))
        for t in self.pending_tasks:
            tstring = t['type']
            if t['type'] == 'mission_plan':
                tstring += ' ('+t['label']+')'
            hb.values.append(KeyValue('-pending task',tstring))

        if self.current_task is None:
            hb.values.append(KeyValue('current_task','None'))
        else:
            hb.values.append(KeyValue('current_task_type',self.current_task['type']))
            if self.current_task['type'] == 'mission_plan':
                hb.values.append(KeyValue('current_task_label',self.current_task['label']))
                hb.values.append(KeyValue('current_task_nav_objective_count',str(len(self.current_task['nav_objectives']))))
                hb.values.append(KeyValue('current_task_nav_objective_index',str(self.current_task['current_nav_objective_index'])))
        self.status_publisher.publish(hb)
               
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
            self.missionManager.publishStatus('Pause')
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
            self.missionManager.publishStatus('Idle')
            rospy.sleep(0.1)
        return 'do-task'


class NextTask(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['idle','mission_plan','goto','hover'])
        
    def execute(self, userdata):
        self.missionManager.current_task = None
        if len(self.missionManager.pending_tasks):
            self.missionManager.current_task = self.missionManager.pending_tasks[0]
            self.missionManager.pending_tasks = self.missionManager.pending_tasks[1:]
            return self.missionManager.current_task['type']
        return 'idle'

class Hover(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['cancelled','exit','pause'])
        self.hover_client = actionlib.SimpleActionClient('hover_action', hover.msg.hoverAction)
        
    def execute(self, userdata):
        if self.missionManager.current_task is not None:
            task = self.missionManager.current_task
            goal = hover.msg.hoverGoal()
            goal.target.latitude = task['latitude']
            goal.target.longitude = task['longitude']
            self.hover_client.wait_for_server()
            self.hover_client.send_goal(goal)
        while self.missionManager.current_task is not None:
            if rospy.is_shutdown():
                return 'exit'
            if self.missionManager.getPilotingMode() != 'autonomous':
                return 'pause'
            self.missionManager.publishStatus('Hover')
            rospy.sleep(0.1)
        self.hover_client.cancel_goal()
        return 'cancelled'

class LineEnded(MMState):
    def __init__(self,mm):
        MMState.__init__(self, mm, outcomes=['mission_plan','next_item'])

    def execute(self, userdata):
        task = self.missionManager.current_task
        if task is not None and task['type'] == 'mission_plan':
            if task['transit_path'] is not None:
                task['transit_path'] = None
            else:
                task['current_path'] = None
                task['current_nav_objective_index'] += 1
            return 'mission_plan'
        return 'next_item'

        
class MissionPlan(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['follow_path','done'])
        
    def execute(self, userdata):
        task = self.missionManager.current_task
        if task is not None:
            if task['current_nav_objective_index'] is None:
                task['current_nav_objective_index'] = 0
            if task['current_nav_objective_index'] >= len(task['nav_objectives']):
                task['current_nav_objective_index'] = None
                return 'done'
            if not 'current_path' in task or task['current_path'] is None:
                self.generatePaths(task)
            return 'follow_path'
        return 'done'

    def generatePaths(self, task):
        path = []
        print task['nav_objectives']
        for p in task['nav_objectives'][task['current_nav_objective_index']]['waypoints']:
            #path.append((p['position']['latitude'],p['position']['longitude']))
            gp = GeoPose();
            gp.position.latitude = p['latitude']
            gp.position.longitude = p['longitude']
            path.append(gp)
        task['current_path'] = path
        # decide if we transit or start line
        task['transit_path'] = None
        if len(task['current_path']) >1:
            start_point = task['current_path'][0]
            next_point = task['current_path'][1]
            if task['do_transit'] and self.missionManager.distanceTo(start_point.position.latitude,start_point.position.longitude) > self.missionManager.waypointThreshold:
                #transit
                transit_path = self.missionManager.generatePathFromVehicle(start_point.position.latitude,start_point.position.longitude, self.missionManager.segmentHeading(start_point.position.latitude,start_point.position.longitude,next_point.position.latitude,next_point.position.longitude))
                task['transit_path'] = transit_path
            task['do_transit'] = True
        
class Goto(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['done','follow_path'])
        
    def execute(self, userdata):
        task = self.missionManager.current_task
        if task is not None:
            if self.missionManager.distanceTo(task['latitude'],task['longitude']) <= self.missionManager.waypointThreshold:
                self.missionManager.current_task = None
                return 'done'
            
            headingToPoint = self.missionManager.headingToPoint(task['latitude'],task['longitude'])
            path = self.missionManager.generatePathFromVehicle(task['latitude'],task['longitude'],headingToPoint)
            task['path'] = path
            task['default_speed'] = self.missionManager.default_speed
            return 'follow_path'
        
class FollowPath(MMState):
    def __init__(self, mm):
        MMState.__init__(self, mm, outcomes=['done','cancelled','exit','pause'])
        self.path_follower_client = actionlib.SimpleActionClient('path_follower_action', path_follower.msg.path_followerAction)
        self.path_planner_client = actionlib.SimpleActionClient('path_planner_action', path_planner.msg.path_plannerAction)
        self.task_complete = False
        
    def execute(self, userdata):
        task = self.missionManager.current_task
        if task is not None:
            if self.missionManager.planner == 'path_follower':
                goal = path_follower.msg.path_followerGoal()
            elif self.missionManager.planner == 'path_planner':   
                goal = path_planner.msg.path_plannerGoal()
            goal.path.header.stamp = rospy.Time.now()
            if task['type'] == 'goto':
                path = task['path']
            if task['type'] == 'mission_plan':
                if task['transit_path'] is not None:
                    path = task['transit_path']
                else:
                    path = task['current_path']
            for s in path:
                #print s
                gpose = GeoPoseStamped()
                gpose.pose = s
                goal.path.poses.append(gpose)
            goal.speed = task['default_speed']
            self.task_complete = False
            if self.missionManager.planner == 'path_follower':
                self.path_planner_client.cancel_goal()
                self.path_follower_client.wait_for_server()
                self.path_follower_client.send_goal(goal, self.path_follower_done_callback, self.path_follower_active_callback, self.path_follower_feedback_callback)
            elif self.missionManager.planner == 'path_planner':
                self.path_follower_client.cancel_goal()
                self.path_planner_client.wait_for_server()
                self.path_planner_client.send_goal(goal, self.path_follower_done_callback, self.path_follower_active_callback, self.path_follower_feedback_callback)

        while self.missionManager.current_task is not None:
            if rospy.is_shutdown():
                return 'exit'
            if self.missionManager.getPilotingMode() != 'autonomous':
                return 'pause'
            if self.task_complete:
                return 'done'
            self.missionManager.publishStatus('FollowPath')
            rospy.sleep(0.1)
        self.path_follower_client.cancel_goal()
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
            smach.StateMachine.add('FOLLOWPATH', FollowPath(missionManager), transitions={'pause':'pause', 'cancelled':'NEXTTASK', 'done':'LINEENDED'})
            smach.StateMachine.add('LINEENDED', LineEnded(missionManager), transitions={'mission_plan': 'MISSIONPLAN', 'next_item':'NEXTTASK'})

        smach.StateMachine.add('AUTONOMOUS', sm_auto, transitions={'pause':'PAUSE', 'exit':'exit'})
    
    sis = smach_ros.IntrospectionServer('mission_manager', sm_top, '/mission_manager')
    sis.start()                                                                            

    sm_top.execute()
    rospy.spin()
    
if __name__ == '__main__':
    main()
