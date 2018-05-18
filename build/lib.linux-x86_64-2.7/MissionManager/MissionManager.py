#!/usr/bin/env python
"""
Created on Mon Nov 20 12:26:52 2017

@author: field
"""

import rospy
import MissionPlan.missionplan
import geodesy.utm

class MissMan_Node():
    
    def __init__(self):
        self.Mission = MissionPlan.missionplan.Mission()
        #print self.Mission;
        
        pass
   
    def readMission(self, filename): 
        '''Read mission file and make list of nav objectives'''
        self.Mission.fromfile(filename)
        #print file
        pass
   
    def convertToRobotCoordinates(self):
        # Convert to PostStamped() message type.
        pass

    def setNextWaypoint(self):
        ''' Send waypoint (in robot coordinates) to MOOS.
        topic: /moos/wpt_updates
        String: points=x1,y1:x2,y2
        '''
        pass
    
    def getDistanceToNextWaypoint(self):
        pass
    
    def getTimeToNextWaypoint(self):
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
        self.readMission("/home/field/catkin_ws/src/mission_plan/src/MissionPlan/mission.txt")
        
        #self.Mission in this test is default missionplan.py output        
        print self.Mission;
    
            
        #In default mission plan, len(self.Mission.plan["NAVIGATION"]) is 2
        #One for waypoint; one for path
        print len(self.Mission.plan["NAVIGATION"])
        
        for x in range(0, len(self.Mission.plan["NAVIGATION"])):
            # print "TEST a"
            
            
            if self.Mission.plan["NAVIGATION"][x].has_key("waypoint"):
                wpt =  self.Mission.plan["NAVIGATION"][x]["waypoint"]
                lat = wpt["nav"]["position"]["latitude"]
                lon = wpt["nav"]["position"]["longitude"]
                print "lat:" + str(lat)
                print "lon:" + str(lon)
                
                utm = geodesy.utm.fromLatLong(lat, lon)
                print "utm: ", utm
            
            elif self.Mission.plan["NAVIGATION"][x].has_key("path"):
                path = self.Mission.plan["NAVIGATION"][x]["path"]
                # print path
                for item in path["nav"]:
                    # print "TEST b"                    
                    # print item
                    if item.has_key("waypoint"):
                        wpt = item["waypoint"]
                        lat = wpt["nav"]["position"]["latitude"]
                        lon = wpt["nav"]["position"]["longitude"]
                        print "lat:" + str(lat)
                        print "lon:" + str(lon)
            
                    utm = geodesy.utm.fromLatLong(lat, lon)
                    print "utm: ", utm
            
            self.convertToRobotCoordinates()
            
            self.setNextWaypoint()
            
            distance_to_go = self.getDistanceToNextWaypoint()
            
            self.getTimeToNextWaypoint()
            
            #while distance_to_go > 3.0:            
            while False:            
                distance_to_go = self.getDistanceToNextWaypoint()            
                self.getTimeToNextWaypoint()
                # Report distance and time to waypoint. TBD
            
            #print "end run"
            #print nav_item
            
            #This is a dictionary
            #self.Mission.plan["NAVIGATION"][0]
            
            #This is a dictionary
            #nav_item
            
            #This is a dictionary
            #self.Mission.plan
            
            #--------------------
            #WHY???
            
            #This is a dictionary
            #self.Mission.plan["DEFAULT_PARAMETERS"]
            
            #This is a list            
            #self.Mission.plan["NAVIGATION"]
            #--------------------
            
            #print self.Mission.plan["NAVIGATION"][0]
            #for a, b in enumerate(nav_item):
            #    print(a,b)
            #for c, d in nav_item.items():
            #    print(c,d)
            
     
    
    
    
    