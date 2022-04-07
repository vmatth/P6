#!/usr/bin/env python
import re
from turtle import pos, width
import rospy
from read_camera.msg import Parcel
import matplotlib.pyplot as plt
import matplotlib as mlp
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import math as ma
from bin_packing.msg import Packing_info
from read_camera.msg import Parcel
from bin_packing.parcel import parcel
from geometry_msgs.msg import Point
from bin_packing.msg import Workspace #workspace msg
from bin_packing.convertTo2DArray import convertTo2DArray #convert function

class first_fit:
    def __init__(self):
        print("init first fit")

        self.workspace_size = Point(0, 0, 0)
        self.height_map = [[]]

        self.pub = rospy.Publisher('/workspace/add_parcel', Packing_info, queue_size=10)
        
        rospy.Subscriber("/parcel_info", Parcel, self.parcel_callback)

        rospy.Subscriber("/workspace/info", Workspace, self.workspace_callback)
    
    def workspace_callback(self, data):
        print("/workspace/info callback")
        #rospy.loginfo(rospy.get_caller_id() + "Receiving data from /workspace/info %s", data)
        self.workspace_size = data.size
        self.height_map = convertTo2DArray(data.height_map, False)
        print("new height_map_ ", self.height_map)

    #Callback function when a new parcel is published to the /parcel_info topic
    def parcel_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Receiving data from /parcel_info %s", data)
        p = parcel(Point(0,0,0), data.size)
        self.start_first_fit(p)

    #Returns if the parcel is in the workspace bounds at (x,y) position
    def parcel_in_range(self, position, parcel):
        size_x = parcel.size.x
        size_y = parcel.size.y
        size_z = parcel.size.z
        pos_x = position.x
        pos_y = position.y
        pos_z = position.z
        #print("Checking if parcel is in range. Size: ", parcel.size, " | Pos: ", position)

        #Check if x is in the workspace
        if (pos_x + size_x) > self.workspace_size.x: #x is out of bounds
            return False

        #Check if y is in the workspace
        if (pos_y + size_y) > self.workspace_size.y:
            return False

        #Check if z is in the workspace
        if (pos_z + size_z) > self.workspace_size.z + 1:
            return False

        return True

    #Returns if the parcel is supported at (x,y) position
    def bottom_supported(self, position, parcel):
        size_x = int(parcel.size.x)
        size_y = int(parcel.size.y)
        pos_x = int(position.x)
        pos_y = int(position.y)

        temp = self.height_map[pos_x][pos_y]
        for x in range(pos_x, pos_x + size_x):
            for y in range(pos_y, pos_y + size_y):
                if temp is not self.height_map[x][y]:
                    return False
        return True

    def first_fit_algorithm(self, parcel):
        ws_x = int(self.workspace_size.x)
        ws_y = int(self.workspace_size.y)

        print("ws size_ ", self.workspace_size)
        print("height_map:", self.height_map)

        for y in range(ws_y):
            for x in range(ws_x):
                z = self.height_map[x][y] #Get z for (x,y) coordinate
                in_range = self.parcel_in_range(Point(x,y,z), parcel)
                print("in_range: ", in_range, " at: ", Point(x,y,z))
                if in_range is True:
                    supported = self.bottom_supported(Point(x,y, 0), parcel)
                    print("supported: ", supported, " at: ", Point(x,y,z))
                    if supported is True:
                        #Publish to a ros topic
                        self.packing_pub(Point(x, y, z), parcel.size)
                        return (x,y,z) #Return x,y coordinate for parcel
            
        return False

    #Goes through all the different rotations for the package
    def start_first_fit(self, p):
        if self.first_fit_algorithm(p) == False:
            p.rotate_parcel('z')
            if self.first_fit_algorithm(p) == False:       
                p.rotate_parcel('y')
                if self.first_fit_algorithm(p) == False:       
                    p.rotate_parcel('z')
                    if self.first_fit_algorithm(p) == False:       
                        p.rotate_parcel('y')
                        if self.first_fit_algorithm(p) == False:       
                            p.rotate_parcel('z')
                            if self.first_fit_algorithm(p) == False:       
                                print("Parcel cannot be packed into the roller cage")

    def packing_pub(self, pos, size):
        msg = Packing_info()

        msg.size = size
        msg.pos = pos

        print(msg)

        self.pub.publish(msg)
        print("Publishing to /packing_info")


def main():
    rospy.init_node('first_fit_algorithm', anonymous=True)
    ff = first_fit() #Create a new instance of the first fit class
    #spin
    rospy.spin()

if __name__ == '__main__':
    main()