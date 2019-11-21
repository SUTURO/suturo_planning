#!/usr/bin/env python
import rospy
import json
from robosherlock_msgs.msg import RSObjectDescriptions

#robosherlock_msgs/RSObjectDescriptions

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    res = json.loads(x)
    rospy.loginfo(rospy.get_caller_id() + "I heard JSON %s", res)
    
def listener():

    rospy.init_node('perception_subscriber', anonymous=True)

    rospy.Subscriber("/perception_pipeline/result_advertiser", RSObjectDescriptions, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
