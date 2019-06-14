#!/usr/bin/python

import rospy, sys, os.path, yaml
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates


class StateAsOdomPublisher(object):
    GAZEBO_TOPIC_NAME = "/gazebo/model_states"

    def __init__(self, state_name, publish_topic_name):
        self.name = state_name
        self.publisher = rospy.Publisher(publish_topic_name, Odometry, queue_size=100)
        # wait for gazebo to start publishing states before attaching a listener to the topic
        try:
            rospy.wait_for_message(self.GAZEBO_TOPIC_NAME, ModelStates)
        except rospy.ROSInterruptException:
            sys.exit("Received shutdown signal...")
        self.listener = rospy.Subscriber(self.GAZEBO_TOPIC_NAME, ModelStates, self.model_states_callback)

    def model_states_callback(self, msg):
        try:
            index = msg.name.index(self.name)
        except ValueError:
            rospy.logdebug("Robot %s state is not published by gazebo." % self.name)
            return
        odom_msg = Odometry()
        odom_msg.pose.pose = msg.pose[index]
        odom_msg.twist.twist = msg.twist[index]
        odom_msg.header.stamp = rospy.Time.now()
        self.publisher.publish(odom_msg)


if __name__=="__main__":
    rospy.init_node("true_pose_from_gazebo", anonymous=True)
    yaml_file = rospy.get_param("~states_to_odom_yaml")

    if yaml_file is not None and os.path.exists(yaml_file):
        with open(yaml_file) as fd:
            args_dict = yaml.safe_load(fd)
        for name in args_dict:
            StateAsOdomPublisher(name, args_dict[name])
    else:
        sys.exit("Parameter 'yaml_file' is not defined or not set to an existing filename.")

    rospy.spin()
