#!/usr/bin/python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
from hsrb_interface import geometry

# Preparation for using the robot functions
robot = hsrb_interface.Robot()
whole_body = robot.get('whole_body')

#Commandline Input
xcoord = float(sys.argv[1])
ycoord = float(sys.argv[2])
zcoord = float(sys.argv[3])
 

# Posture to touch target
target_to_hand = geometry.pose(x=xcoord, y=ycoord,z=zcoord)


try:
        # Transit to initial posture
        whole_body.move_to_neutral()
        # Move the hand to target
        whole_body.move_end_effector_pose(target_to_hand, ref_frame_id='head_rgbd_sensor_rgb_frame')
        # Transit to initial posture
        whole_body.move_to_neutral()
        
except:
        rospy.logerr('failed to touch')
        sys.exit()

