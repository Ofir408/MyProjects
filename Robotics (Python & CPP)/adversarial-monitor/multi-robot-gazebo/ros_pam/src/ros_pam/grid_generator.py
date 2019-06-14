#!/usr/bin/python

import argparse, random
from PIL import Image
import rospy


## Parser cmd-line arguments, expecting to see the dimensions, mode and cell size of an image,
#  as well as the filename to save the generated image to.
#  @returns argparse.Namespace object that allows extraction of the arguments as named properties.
def parse_args():
    # in order to reserve '-h' to '--height', create the parser with add_help=False
    # and construct the help argument manually.
    parser = argparse.ArgumentParser(description="Generate a random image with the given mode and dimensions.", add_help=False)
    parser.add_argument("--help", action='help', default=argparse.SUPPRESS, help="show this help message and exit")
    parser.add_argument("-w", "--width", type=int, required=True, help="The width of the image in pixels")
    parser.add_argument("-h", "--height", type=int, required=True, help="The height of the image in pixels")
    parser.add_argument("-c", "--cell-size", type=int, default=1, help="The size of a grid cell in pixels (for better visualization of the grid)")
    parser.add_argument("-m", "--mode", choices=["L", "RGB"], default="L",
                        help="A string indicating the image mode. 'L' stands for greyscale, 'RGB' for standard RGB mode.")
    parser.add_argument("-f", "--file", required=True, help="The filename to save the image to.")
    return parser.parse_args()


## Generates a random grid as image.
#  Saves that image to args.filename.
def generate_grid(args):
    image = Image.new(args.mode, (args.width, args.height))
    # get the number of cells in a row and in a column
    rows = args.width / args.cell_size
    cols = args.height / args.cell_size
    # get the number of bands and set the random_value function
    # to return a single random value for single-band (grey-scale)
    # and a tuple of random value per band for other modes (3 for RGB, 4 for CMYK, etc)
    bands = image.getbands()
    if len(bands) == 1:
        random_value = random.randint
    else:
        random_value = lambda x, y: tuple(random.randint(x, y) for band in bands)
    # generate a random matrix of size rowsXcols
    cell_values = [[random_value(0, 255) for col in xrange(cols)] for row in xrange(rows)]
    # set the pixel value of all pixels, based on the cell they belong to
    for i in xrange(args.width):
        for j in xrange(args.height):
            image.putpixel((i, j), cell_values[i / args.cell_size][j / args.cell_size])
    image.save(args.file)


if __name__ == "__main__":
    rospy.init_node("grid_generator", anonymous=True)
    args = parse_args()
    generate_grid(args)
    rospy.loginfo("Saved random image to %s" % args.file)
    rospy.signal_shutdown("Finished")

