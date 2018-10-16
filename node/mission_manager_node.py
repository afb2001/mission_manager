#!/usr/bin/env python

import rospy
import mission_manager

if __name__ == '__main__':
    a = mission_manager.MissionManager_Node()
    a.run()

