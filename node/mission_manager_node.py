#!/usr/bin/env python

import rospy
import mission_manager

if __name__ == '__main__':
    node = mission_manager.MissionManager_Node()
    node.run()

