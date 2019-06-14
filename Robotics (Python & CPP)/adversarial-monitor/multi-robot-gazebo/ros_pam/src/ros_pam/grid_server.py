#!/usr/bin/python

"""
    Similarly to map_server (though with reduced functionality), this node will parse a grid file of various formats
    (see http://infohost.nmt.edu/tcc/help/pubs/pil/formats.html for a list of supported formats and extensions),
    and will provide it in a nav_msgs::OccupancyGrid message over a latched topic.
    
    The main difference from map_server is that this node will also provide services to update cells in the grid,
    reload the grid from another file, and save the grid, on runtime.
    
    Additionally, OccupancyGrid data will contain values ranging from 0-255, which represent a threat probability.
    
"""

import sys, rospy, yaml, copy, os.path
from PIL import Image
from nav_msgs.msg import OccupancyGrid
from ros_pam.srv import *


## Data structure for GridServer.
#  Holds the grid as loaded from the given file allowing update/reload and save methods.
class GridStructure(object):
    def __init__(self, filename, resolution, negate=False):
        self.filename = filename
        self.resolution = resolution
        self.negate = negate
        self.load_image()

    ## loads an image stored in self.filename
    def load_image(self):
        rospy.loginfo("Loading threats from %s" % self.filename)
        # Load the image and convert it to greyscale.
        # If the image is in RGB, the conversion is done with the following formula:
        # L = R * 299/1000 + G * 587/1000 + B * 114/1000
        # TODO: add support for the rest of conversion modes (e.g. linear approx)
        self.grid = Image.open(self.filename).convert("L")
        if self.negate:
            self.grid = self.grid.point(lambda x: 255 - x)
        self.load_time = rospy.Time.now()
        rospy.loginfo("Loaded %dx%d image file as a threat grid, with resolution %f (negate is %s)" % (self.grid.size[0], self.grid.size[1], self.resolution, self.negate))

    ## Saves the current grid to a given filename (default to the original filename)
    def save(self, filename=None):
        if filename is None:
            filename = self.filename
        if self.negate:
            temp_copy = self.grid.point(lambda x: 255 - x)
        else:
            temp_copy = self.grid.copy()
        rospy.loginfo("Saving threats map to %s" % filename)
        temp_copy.save(filename)

    ## Updates a grid's pixel to the given value (and updates map accordingly)
    #  @param indices - an (i,j) tuple
    #  @param value   - a 0-255 range int
    def update_grid_pixel(self, indices, value):
        self.grid.putpixel(indices, value)
        self.load_time = rospy.Time.now()

    @property
    def data(self):
        return list(self.grid.getdata())

    @property
    def dimensions(self):
        return self.grid.size

    @property
    def occupancy_msg(self):
        msg = OccupancyGrid()
        msg.info.width, msg.info.height = self.dimensions
        msg.info.map_load_time = self.load_time
        msg.info.resolution = self.resolution
        # OccupancyGrid accepts values in the range of [-127,127],
        # so we scale the data by half to get a range of [0,127] and match the types, while avoiding negative values.
        msg.data = map(lambda x: x / 2, self.data)
        return msg


## The main class. 
#  Provides the necessary services and publishers.
#  Uses GridParser as the data structure.
class GridServer(object):
    def __init__(self, filename, resolution, negate):
        self.filename = filename
        self.resolution = resolution
        self.negate = negate
        self.grid_publisher = rospy.Publisher('threat_grid', OccupancyGrid, queue_size=1, latch=True)
        self.update_grid_service = rospy.Service('update_grid_cell', UpdateGridCell, self.update_grid_cell)
        self.save_grid_service = rospy.Service('save_grid', SaveGrid, self.save_grid)
        self.reload_grid_service = rospy.Service('reload_grid', ReloadGrid, self.reload_grid)
        self.load_grid()

    ## Publishes the grid and the map to the matching topics.
    def publish(self):
        grid_msg = copy.deepcopy(self.grid.occupancy_msg)
        grid_msg.header.stamp = rospy.Time.now()
        # TODO: what about seq and frame_id fields of the header message?
        self.grid_publisher.publish(grid_msg)

    ## Creates the GridMapStructure and publishes the grid and the map
    def load_grid(self):
        self.grid = GridStructure(self.filename, self.resolution, self.negate)
        self.publish()

    ## Service handler to update a specific cell in the threat grid to a given value.
    #  @param msg - an UpdateGridCell service msg
    #  @returns - the UpdateGridCellResponse response message
    def update_grid_cell(self, msg):
        rospy.logdebug("Updating threat map (%d,%d) cell to %d" % (msg.row, msg.col, msg.value))
        self.grid.update_grid_pixel((msg.row, msg.col), msg.value)
        self.publish()
        return UpdateGridCellResponse(response=True)

    ## Service handler to save the threat grid to a given file.
    #  When the filename is None, this will cause overwriting the file from which the grid was originally loaded.
    #  @param msg - an SaveGrid service msg
    #  @returns - the SaveGridResponse response message
    def save_grid(self, msg):
        rospy.logdebug("Saving threat map to %s" % msg.filename)
        self.grid.save(msg.filename)
        return SaveGridResponse(response=True)

    ## Service handler to reload the threat grid from a given file.
    #  When the filename is None, the grid will be reloaded from its original file.
    #  @param msg - an ReloadGrid service msg
    #  @returns - the ReloadGridResponse response message
    def reload_grid(self, msg):
        if msg.file is not None:
            self.filename = msg.file
        if msg.resolution is not None:
            self.resolution = msg.resolution
        rospy.logdebug("Reloading threat map from %s" % self.filename)
        self.load_grid()
        return ReloadGridResponse(response=True)


if __name__ == "__main__":

    rospy.init_node("grid_server", anonymous=True)

    yaml_file = rospy.get_param("~yaml_file")

    if yaml_file is not None and os.path.exists(yaml_file):
        with open(yaml_file) as fd:
            args_dict = yaml.safe_load(fd)
            filename = os.path.join(os.path.dirname(yaml_file), args_dict["image"])
            resolution = args_dict["resolution"]
            negate = args_dict.get("negate", False)
    else:
        sys.exit("Parameter 'yaml_file' is not defined or not set to an existing filename.")

    GridServer(filename, resolution, negate)
    rospy.spin()
