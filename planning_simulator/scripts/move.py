#!/usr/bin/env python
# -*- coding: utf-8 -*-
from hsrb_interface import Robot
import rospy
import math
from std_srvs.srv import Trigger, TriggerRequest

rospy.init_node('move', anonymous=True)

# Preparation to use robot functions
robot = Robot()
base = robot.try_get('omni_base')
tts = robot.try_get('default_tts')
whole_body = robot.try_get('whole_body')

if __name__=='__main__':
    # Move to initial posture
    try:
	base.go_abs(1.0, -2.0, -1.0, 180.0)
#        base.go_abs(0.6, -0.2, 0, 180.0)	
        whole_body.move_to_joint_positions({'head_tilt_joint': -0.3})
    except:
        rospy.logerr('Fail move_to_neutral')
    rospy.wait_for_service('/perception_pipeline/trigger')
    srv = rospy.ServiceProxy('/perception_pipeline/trigger',Trigger)
    try:
       msg = TriggerRequest()
       result = srv(msg)
       print result;
    except rospy.ServiceException as exc:
	print "Service call to perception_pipeline failed.";
    
    
