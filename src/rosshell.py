#!/usr/bin/env python
import os
import sys
import roslib; roslib.load_manifest("rosshell")
import rospy

print sys.argv

ret = 0
for command in range(1,len(sys.argv)):
	ret=ret|os.system(sys.argv[command])

ret=ret>>8
rospy.signal_shutdown('finish')
sys.exit(ret) 
