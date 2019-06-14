#!/usr/bin/python

import rospy, sys, os.path, yaml
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point

import tf



"""
Convert each tf lookuptransform to Odom msg type, and send it by the relevant topics.
For example, monitor node in ros_pam pacakge will get Odom, which uses in order to track the robot 
for checking if the robot was caught in a threat.
"""
class OptitrackOdomPublisher(object):
    WORLD = "/world"

    # the optitrack gives frames info between /world /{ours}
    def __init__(self, robot_name, optitrack_robot):
        self.optitrack_robot_name = optitrack_robot
        self.pub_odom_topic_name = "/" + robot_name + "/optitrack_data"
        self.odom_publisher = rospy.Publisher(self.pub_odom_topic_name, Odometry, queue_size=100)


    """
    No need to use quaternion to detect robot grid cell from optitrack data.
    Therefore, gets only position (x, y, z) of the robot location. 
    gets tf lookupframe result.
    Translate it to Odom msg and then return it. 
    """
    @staticmethod
    def from_lookup_frame_to_odom(position):
        #print "current robot received position is: " + str(position)
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        # set the position
        quat_tf = [0, 1, 0, 0]

        quat_msg = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])

        quaternion = Quaternion(0.0,0.0,0.0,0.0)
        odom_msg.pose.pose.orientation = quat_msg
        odom_msg.pose.pose.position = Point (position[0], position[1], position[2]) # quaternion is None. See remark below.
        return odom_msg


    def publish_odom_msg(self, translated_odom_msg):
        self.odom_publisher.publish(translated_odom_msg)
	print "sent"
	#print "odom msg to be sent is: " + str(translated_odom_msg)
        #print "translated_odom_msg sent to topic: " + self.pub_odom_topic_name



    def start_optitrack_tracker(self):
        rate = rospy.Rate(10.0)
        listener = tf.TransformListener()

        while not rospy.is_shutdown():
            try:
                (position, quaternion) = listener.lookupTransform(OptitrackOdomPublisher.WORLD, self.optitrack_robot_name, rospy.Time(0))
                print ("got new tf transform")

                translated_odom_msg = OptitrackOdomPublisher.from_lookup_frame_to_odom(position)
                self.publish_odom_msg(translated_odom_msg)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()


if __name__=="__main__":
    print "optitrack_odom_pub STARTED"
    rospy.init_node("optitrack_pub", anonymous=True)


    if len(sys.argv) < 2:  # because the first arg is the program name.
        print ("Invalid Argument number, did you pass the yaml_file? ")
        exit(-1)

    # the yaml file composes from dict of tuples, each tuple is: (robot_name, optitrack_robot_name)
    yaml_file = sys.argv[1]
    print "yaml_file path is: " + yaml_file
    if yaml_file is not None and os.path.exists(yaml_file):
        with open(yaml_file) as fd:
            args_dict = yaml.safe_load(fd)
        for current_robot_name in args_dict:
            current_optitrack_robot_name = args_dict[current_robot_name]
            rospy.loginfo ("current_robot_name is: " + current_robot_name + " , optitrack_name is: " + current_optitrack_robot_name)
            current_publisher = OptitrackOdomPublisher(current_robot_name, current_optitrack_robot_name)
            current_publisher.start_optitrack_tracker()

    else:
        sys.exit("Parameter 'yaml_file' is not defined or not set to an existing filename!")

