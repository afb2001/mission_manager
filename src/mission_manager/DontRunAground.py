#!/usr/bin/env python
"""
Created on Mon Nov 20 12:26:52 2017

@author: ldavis
"""

import rospy
import json
from mission_manager.msg import BehaviorControl
from std_msgs.msg import String, Float32, Int32, Bool


class DontRunAground_Node():
    
    def __init__(self, default = True):
        
        rospy.init_node("DontRunAground")
        
        #print "**DON'T RUN AGROUND, BEN behavior is active.**"
        
        # TODO: depthThreshold to be set in parameters file?
        #self.minDepth = rospy.get_param(mindepth)
        self.minDepth = 4.0
        
        self.depth = Float32
        self.wptIndex = Int32
        self.wptType = String   
        self.isActive = False
        
        self.shallowWaterStatus = String()
        self.shallowWaterStatus.data = "OK"
        # TODO: Threshold possibly set thresholds in parameters?
        # Set minimum distance to waypoint
        #self.shallowWaterDanger = False
        self.shallowWaterNegativeCount = 0
        self.shallowWaterPositiveCount = 0
        # If ASV receives this many consecutive negative shallow warnings,
        # reset positive shallowWater counts
        self.shallowWaterNegativeCountThreshold = 5
        # If ASV receives this many consecutive positive shallow warnings, 
        # deviate path and reset negative shallowWater counts
        self.shallowWaterPositiveCountThreshold = 5
        
        
        # Listen to /behavior_ctrl to determine when "Don't Run Around, Ben" behavior becomes Active.
        rospy.Subscriber('/behavior_ctrl', BehaviorControl, self.behavior_ctrl_callback, queue_size = 1)
        # TODO: May need to change following message types -- unsure if correct.
        # Receive up-to-date depth 
        rospy.Subscriber('/depth', Float32, self.depth_callback, queue_size = 1)
        
        # To publish shallow water warnings. Mission Manager to subscribe to this topic and respond accordingly.
        self.shallow_water = rospy.Publisher('/shallow_water_warning', String, queue_size = 10)
    
    
    def behavior_ctrl_callback(self, behaviorCtrl_msg):
        # Check to see whether behaviorCtrl_msg contains "Don't Run Around, Ben" beavior set to Active.
        if behaviorCtrl_msg.Name == 'dont_run_aground_ben' and behaviorCtrl_msg.IsActive:
            parametersDict = json.loads(behaviorCtrl_msg.Parameters)
            
            if parametersDict["dont_run_aground_ben"].has_key("mindepth_m"):
                self.minDepth = parametersDict["dont_run_aground_ben"]["mindepth_m"]
            
            self.isActive = True
            print "DRAB Activated"
            
        else: 
            self.isActive = False
            # Reset shallowWaterStatus
            self.shallowWaterStatus.data = "OK"
            print "DRAB Deactivated"
                
            #self.dontRunAgroundBen()
    
    
    def depth_callback(self, depth_msg):
        # Save up-to-date depth
        self.depth = depth_msg.data
    

    def run(self):
        '''
        Implements "Don't Run Aground, Ben!" behavior. Utilizes /depth topic to compare with
        minDepth. If current depth is less than minDepth, return WARNING to mission manager.
        '''
        
        #print "In dontRunAgroundBen function."
        
        while not self.isActive:
            rospy.sleep(0.5)
            if rospy.is_shutdown():
                break
        
        if self.depth >= self.minDepth:
            self.shallowWaterNegativeCount += 1
            self.shallowWaterPositiveCount = 0
            
            if self.shallowWaterNegativeCount >= self.shallowWaterNegativeCountThreshold:
                self.shallowWaterStatus.data = "OK"
                print "OK"
            
        else:
            self.shallowWaterPositiveCount += 1
            self.shallowWaterNegativeCount = 0
            
            '''
            if self.shallowWaterPositiveCount >= (self.shallowWaterPositiveCountThreshold * 2) or \
            self.depth < (self.minDepth * 0.5):
            '''
            if self.depth < (self.minDepth *0.75):
                self.shallowWaterStatus.data = "EMERGENCY"
                print "EMERGENCY"
                #print "PERSISTENT SHALLOW WATER READINGS"                
                #print "ENTER EMERGENCY MODE AND TAKE CORRECTIVE ACTION IMMEDIATELY."
            elif self.shallowWaterPositiveCount >= self.shallowWaterPositiveCountThreshold:
                self.shallowWaterStatus.data = "WARNING"
                print "WARNING"
                #print "SHALLOW WATER DETECTED."  
            
        self.shallow_water.publish(self.shallowWaterStatus)
        
