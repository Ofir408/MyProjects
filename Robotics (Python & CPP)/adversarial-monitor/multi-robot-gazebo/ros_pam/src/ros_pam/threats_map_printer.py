#!/usr/bin/python
import os
import errno
from PIL import Image
from PIL import ImageFont
from PIL import ImageDraw

"""
Prints the threats map with the probabilities on each cell.
Each probability represents a threat in a cell on the grid.
"""


class ThreatsMapPrinter:
    PACKAGE_NAME = "log_files"
    BASE_PATH = "/src/logger/" + PACKAGE_NAME

    def __init__(self, threat_map, norm_factor):
        self.threat_map = threat_map
        self.norm_factor = norm_factor
        self.__file_absolute_path = ThreatsMapPrinter.__get_catkin_ws_path() + self.BASE_PATH + "/" + "Threats Probabilities.txt"

    def write_threats_prob(self):
        file_des = self.__create_file_in_package()
        robot_path_str = self.__get_str_from_threats()

        file_des.write(robot_path_str)
        file_des.close()
        """
         L = [[1, 2, 3, 1],
             [4, 5, 6, 9],
             [7, 8, 9, 4]]
        L = self.threat_map.data
        to_insert = []
        for x in xrange(60):
            m = []
            for c in xrange(60):
                m.append(c)
            to_insert.append(m)

        print str(to_insert)
        L = to_insert
        size = 360
        img = Image.new("RGB", (size, size), 'white')
        draw = ImageDraw.Draw(img)
        font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeMono.ttf", 48)
        for i in range(0, size, size // 60):
            for j in range(0, size, size // 60):
                print(i // 100, j // 100)
                draw.text((i + size // 9, j + size // 9), str(L[i // 100][j // 100]), (0, 0, 0), font=font)
        img.save(ThreatsMapPrinter.__get_catkin_ws_path() + self.BASE_PATH + '/numbers.jpg')

        """

    """
    returns a string from a point list
    """

    def __get_str_from_threats(self):
        threats_str = "Verify that text-wrapping is disabled to see the full grid\n\n"
        width = self.threat_map.info.width
        height = self.threat_map.info.height

        for row_num in xrange(height):
            for col_num in xrange(width):
                prob = self.threat_map.data[row_num * height + col_num]
                after_normalization = "%.2f" % (prob / self.norm_factor)  # just 2 numbers after the point.
                threats_str += after_normalization + " "
            threats_str += "\n"

        return threats_str

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

    @staticmethod
    def __get_catkin_ws_path():
        str_to_search = "catkin_ws"
        dirname, filename = os.path.split(os.path.abspath(__file__))
        current_path = dirname + "/" + filename

        if str_to_search not in current_path:
            str_to_search = "hamster_ws"  # lab environment.
        first_catkin_ws_inx = current_path.find(str_to_search)
        str_to_return = current_path[0: first_catkin_ws_inx + len(str_to_search)]
        if first_catkin_ws_inx < 0:
            str_to_return = "/home/pi/hamster_ws"
        print "__get_catkin_ws_path returned: " + str_to_return
        return str_to_return
