#!/usr/bin/env python

from numpy import append
from lidar.srv import lidar,lidarResponse
import sys
import rospy
import PIL
from PIL import Image, ImageDraw
import pathlib
import math

def plot_reading_on_map(centerX, centerY, lidar_reading, map):
    '''This plots the collected lidar data on map'''
    for i, reading in enumerate(lidar_reading):

        try:
            '''As the angle given by lidar is integral,
            if we try to map all the points where lidar's light could pass,
            we see that the generated map doesnt show the points between two light rays.
            So here I have collected two consecutive points which lidar has detected,which are
            expected to be at a very close distance,and then collect all the points inside the
            triangle formed by these points-
            1.point where the sensor is
            2 and 3.The close consecutive points
            Credits-MATH calculus course :)
            '''
            i0 = reading[0]
            r0 = reading[1]

            i1 = lidar_reading[i+1][0]
            r1 = lidar_reading[i+1][1]

            x0 = int(r0*math.cos(i0*math.pi/180)+centerX)
            y0 = int((r0*math.sin(i0*math.pi/180)+centerY))

            x1 = int(r1*math.cos(i1*math.pi/180)+centerX)
            y1 = int((r1*math.sin(i1*math.pi/180)+centerY))

            draw = ImageDraw.Draw(map)
            draw.polygon([(centerX, centerY), (x0, y0),
                         (x1, y1)], fill=255, outline=255)

        except:
            # to avoid encountering error at the edges
            continue

def lidar_client(x, y):
    rospy.wait_for_service('scan')
    try:
        lid_scan = rospy.ServiceProxy('scan', lidar)
        resp1 = lid_scan(x, y)
        lid_scan=[]
        for i,s in enumerate(resp1.lidar_array):
            if(i%2):
                lid_scan.append((resp1.lidar_array[i-1],s))
        return lid_scan
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
    plot_reading_on_map(80,140,lidar_client(80,140),map)
    plot_reading_on_map(0,320,lidar_client(0,320),map)
    plot_reading_on_map(200,0,lidar_client(200,0),map)
    plot_reading_on_map(200,280,lidar_client(200,280),map)
    plot_reading_on_map(280,260,lidar_client(280,260),map)
    map.save("map.jpg")
    
    
    
    