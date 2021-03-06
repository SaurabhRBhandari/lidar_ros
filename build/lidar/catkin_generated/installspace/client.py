#!/usr/bin/env python3

from lidar.scripts.utils import plot_reading_on_map
from lidar.srv import lidar,lidarResponse
import sys
import rospy
import PIL
from PIL import Image, ImageDraw
import pathlib
import math


def lidar_client(x, y):
    rospy.wait_for_service('scan')
    try:
        lid_scan = rospy.ServiceProxy('scan', lidar)
        resp1 = lid_scan(x, y)
        return list(resp1.lidar_array),x,y
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# This stores the map created by lidar
map = PIL.Image.new(mode="1", size=(400,400))

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
    print("Requesting scan at (%s,%s)"%(x, y))
    lid_scan,x,y=lidar_client(x,y)
    plot_reading_on_map(x,y,lid_scan,map)
    print("done generating the map")