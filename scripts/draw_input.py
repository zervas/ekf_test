from matplotlib import pyplot as plt
from matplotlib.image import BboxImage
from matplotlib.transforms import Bbox, TransformedBbox
from matplotlib import transforms
from scipy import ndimage
import math
import time

# Load images.
redMarker = plt.imread('robot.png')

# Data
redX = [1, 2, 3, 4]
redY = [3, 2, 3, 4]

# Create figure
fig = plt.figure()
ax = fig.add_subplot(111)

# Plots an image at each x and y location. 
def plotRobot(xData, yData, radians):
    x = xData - 0.18
    y = yData - 0.25
    deg = math.degrees(radians)
    im = plt.imread('robot.png')
    im = ndimage.rotate(im, deg)
    bb = Bbox.from_bounds(x,y,0.5,0.5)  
    bb2 = TransformedBbox(bb,ax.transData)
    bbox_image = BboxImage(bb2,
                        norm = None,
                        origin=None,
                        clip_on=False)

    bbox_image.set_data(im)
    ax.add_artist(bbox_image)

plotRobot(0.1005429227, 0.0999857089, -0.0002892237)


# Set the x and y limits
ax.set_ylim(0,6)
ax.set_xlim(0,6)

plt.show()