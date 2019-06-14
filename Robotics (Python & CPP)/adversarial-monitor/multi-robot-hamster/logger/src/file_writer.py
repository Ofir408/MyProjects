#!/usr/bin/python

import rospy
from ros_pam.msg import TriggerReport
import os
import errno


class FileWriter(object):
    THREAT_TOPIC_NAME = "adversarial_trigger_report"
    PACKAGE_NAME = "log_files"
    FILE_NAME = "log"
    BASE_PATH = "/src/logger/" + PACKAGE_NAME  # will concatenate between ../catkin_ws to BASE_PATH

    def __init__(self):
        rospy.loginfo("started")
        # example: /home/ofir/catkin_ws/src/logger/launch
        #self.file_absolute_path = os.getcwd() + "/../" + self.PACKAGE_NAME + "/" + self.FILE_NAME
        self.file_absolute_path = FileWriter.get_catkin_ws_path() + self.BASE_PATH + "/" + self.FILE_NAME + ".txt"
        print ("file_abs_path is: " + self.file_absolute_path)

        self.file_des = self.create_file_in_package()
        self.threat_subscriber = rospy.Subscriber(self.THREAT_TOPIC_NAME, TriggerReport, self.got_new_report_callback)

    def __del__(self):
        if self.file_des is not None:
            self.file_des.close()

    # write new report to the file.
    def write_new_report(self, time, robot_name, prob, cell):
        if not os.path.isfile(self.file_absolute_path) or self.file_des is None:
            raise FileNotFoundError("abs path: " + self.file_absolute_path + ", not found")

        self.file_des.write("Time: " + str(time) + ", Robot_name: " + robot_name + ", cell: " + cell + ", prob: " + str(prob) + "\n")
        self.file_des.flush()  # because we want to see the changes Immediately.


    # writes the new report to the file.
        # Header header
        # string robot_ns
        # float32 threat_probability
    def got_new_report_callback(self, msg):
        time = msg.header.stamp
        robot_name = msg.robot_ns
        prob = msg.threat_probability
	cell = msg.caught_location
        rospy.loginfo("[New Log msg]: time " + str(time) + " , robot_name: " + robot_name + ", cell: " + cell + " , prob: " + str(prob))
        self.write_new_report(time, robot_name, prob, cell)

    # creates the file that the logger writes to.
    # if the library or the file doesn't exist, create them.
    ## NOTE: the file path is: ...(absolute path)/logger/log_files/FILE_NAME
    def create_file_in_package(self):
        if self.file_absolute_path is None:
            raise TypeError("FileWriter:create_file, file_absolute_path is None")

        if not os.path.exists(os.path.dirname(self.file_absolute_path)):
            try:
                os.makedirs(os.path.dirname(self.file_absolute_path))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        fd = open(self.file_absolute_path, "w")
        return fd

    @staticmethod
    def get_catkin_ws_path():
        str_to_search = "catkin_ws"
        current_path = os.getcwd()
	print "os.getcwd() returned: " + os.getcwd()
        if str_to_search not in current_path:
            str_to_search = "hamster_ws"  # lab environment.

        first_catkin_ws_inx = current_path.find(str_to_search)
        str_to_return = current_path[0 : first_catkin_ws_inx + len(str_to_search)]
        return str_to_return



if __name__== "__main__":
    rospy.init_node("file_writer", anonymous=True)
    FileWriter() # start the logger.
    rospy.spin()

