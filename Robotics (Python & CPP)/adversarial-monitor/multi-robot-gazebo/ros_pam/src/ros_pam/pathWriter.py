#!/usr/bin/env python
import os
import errno
import rospy

"""
Gets the that the robot did and write the path to a text file.
"""


class PathWriter:

    PACKAGE_NAME = "log_files"
    BASE_PATH = "/src/logger/" + PACKAGE_NAME + "/Robots Paths"  # will concatenate between ../catkin_ws to BASE_PATH


    """
    robot_path is a list of points - each one represents robot's location in the grid.
    """
    def __init__(self, robot_name, robot_path):
        self.__robot_path = PathWriter.__remove_repeats(robot_path)
        self.__file_absolute_path = PathWriter.__get_catkin_ws_path() + self.BASE_PATH + "/" + robot_name + ".txt"


    """
    returns a string that represents the path of the robot in the grid. 
    """
    def __from_list_to_str(self):
        robot_path_str = ""
        for current_point in self.__robot_path:
            robot_path_str += str(current_point) + "\n"

        # remove the last \n
        if len(robot_path_str) > 0:
            robot_path_str = robot_path_str[:-1]
        return robot_path_str

    def __create_file_in_package(self):
        if self.__file_absolute_path is None or "":
            raise TypeError("pathWriter:create_file, file_absolute_path is None")

        if not os.path.exists(os.path.dirname(self.__file_absolute_path)):
            try:
                os.makedirs(os.path.dirname(self.__file_absolute_path))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        fd = open(self.__file_absolute_path, "w")
        return fd

    """
    Writes the path of the robot to a text file that located on {file_path}
    """
    def write_path(self):
        file_des = self.__create_file_in_package()
        robot_path_str = self.__from_list_to_str()

        file_des.write(robot_path_str)
        file_des.close()

    @staticmethod
    def __get_catkin_ws_path():
        str_to_search = "catkin_ws"
        current_path = os.path.dirname(os.path.realpath(__file__))

        print "os.getcwd() is: " + current_path
        if str_to_search not in current_path:
            str_to_search = "hamster_ws"  # lab environment.
        first_catkin_ws_inx = current_path.find(str_to_search)
        str_to_return = current_path[0: first_catkin_ws_inx + len(str_to_search)]
        if first_catkin_ws_inx < 0:
            str_to_return = "/home/pi/hamster_ws"
        print "__get_catkin_ws_path returned: " + str_to_return
        return str_to_return

    """
    Remove repeats from robot's path.
    For example, if due to multiple optitrack callbacks we got [ (1, 2), (1, 2), (2, 3) ]
    So the path is [ (1, 2), (2, 3) ]. 
    """
    @staticmethod
    def __remove_repeats(path):
        without_repeats = []
        prev = None
        for current_point in path:
            if prev != current_point:
                without_repeats.append(current_point)
                prev = current_point
        print "path_without_repeats is: " + str(without_repeats)
        return without_repeats

