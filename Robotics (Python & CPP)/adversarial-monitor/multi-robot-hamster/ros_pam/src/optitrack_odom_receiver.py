#!/usr/bin/python

import rospy, sys, os.path, yaml
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import tf


class OptitrackOdomSubscriber(object):


    # the optitrack gives frames info between /world /{ours}
    def __init__(self, robot_name):
        self.sub_odom_topic_name = "/" + robot_name + "/optitrack_data"
        self.odom_sub = rospy.Subscriber(self.sub_odom_topic_name, Odometry, self.my_callback)
        rospy.spin()
	print ("Set listener")


    def my_callback(self, msg):
       print ("ON Callback called ! ")



if __name__=="__main__":
    print "optitrack_odom_sub STARTED"
    rospy.init_node("optitrack_sub", anonymous=True)

    o = OptitrackOdomSubscriber("agent3")
    

