#!/usr/bin/env python
"""
Spyder Editor

This is a temporary script file.
"""

#
# gps_sim_node
# 
# A GPS simulator node, that reads output from robot simulation
# and converts to standard GPS NMEA strings for testing. 
#
# Val Schmidt
# Center for Coastal and Ocean Mapping
# University of New Hampshire
# Copyright 2017, All rights reserved.

import MissionManager.MissionManager

import MissionManager.untitled0

import rospy

if __name__ == '__main__':
    a = MissionManager.untitled0.MissMan_Node()
    
    while not rospy.is_shutdown():
        #g = MissionManager.MissionManager.MissMan_Node()
    
        
      
    
        try:
            #g.init_service()
        
            #g.run()
            a.run()
        
            #TEST
            #g.readMission(mission.txt)
        
        except rospy.ROSInterruptException:
            print "Error of unknown origin"
            pass

