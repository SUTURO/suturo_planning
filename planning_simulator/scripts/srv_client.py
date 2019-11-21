#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Trigger, TriggerRequest

rospy.init_node('p_p_service_client');

if __name__=='__main__':
    rospy.wait_for_service('/perception_pipeline/trigger')
    srv = rospy.ServiceProxy('/perception_pipeline/trigger',Trigger)
    print srv
    try:
       msg = TriggerRequest()
       result = srv(msg)
       print result;
    except rospy.ServiceException as exc:
	print "Service call to perception_pipeline failed.";
