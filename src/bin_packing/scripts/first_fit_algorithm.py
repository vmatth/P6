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



#Class for the packing system. Defines the workspace plots
class workspace:

    #Init function for the packing setup. Creates the workspace & height map plots. Workspace sizes are in [cm]. 
    def __init__(self, x, y, z):
        #rospy.Subscriber("/parcel_info", Parcel , self.parcel_callback)

        self.parcels = [] #Stores all of the parcels in the workspace

        self.workspace_size = (x, y, z) #Save the workspace size in a variable

        self.height_map_array = [[0 for i in range(x)] for j in range(y)] 





class first_fit:
    def __init__(self, ws):
        print("init first fit")
        print("workspace size: ", ws.workspace_size)
        self.ws = ws
        self.pub = rospy.Publisher('/packing_info', Packing_info, queue_size=10)
        rospy.Subscriber("/parcel_info", Parcel, self.parcel_callback)

        #subscriber 
    
    #Callback function when a new parcel is published to the /parcel_info topic
    def parcel_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        p = parcel((0,0,0), (int(data.width), int(data.height), int(data.depth)))
        self.start_first_fit(p)

    #def place_parcel(self, pos, parcel):
        #publish to a topic

    #Returns if the parcel is in the workspace bounds at (x,y) position
    def parcel_in_range(self, position, parcel):
        size_x = parcel.size[0]
        size_y = parcel.size[1]
        size_z = parcel.size[2]
        pos_x = position[0]
        pos_y = position[1]
        pos_z = position[2]
        print("Checking if parcel is in range. Size: ", parcel.size, " | Pos: ", position)

        print("ws size_ ", self.ws.workspace_size)
        #Check if x is in the workspace
        if (pos_x + size_x) > self.ws.workspace_size[0]: #x is out of bounds
            return False

        #Check if y is in the workspace
        if (pos_y + size_y) > self.ws.workspace_size[1]:
            return False

        #Check if z is in the workspace
        if (pos_z + size_z) > self.ws.workspace_size[2] + 1:
            return False

        return True

    #Returns if the parcel is supported at (x,y) position
    def bottom_supported(self, position, parcel):
        size_x = int(parcel.size[0])
        size_y = int(parcel.size[1])
        pos_x = int(position[0])
        pos_y = int(position[1])

        print("Checking if parcel is SUPPORTED. Size: ", parcel.size, " | Pos: ", position)

        temp = self.ws.height_map_array[pos_x][pos_y]
        for x in range(pos_x, pos_x + size_x):
            for y in range(pos_y, pos_y + size_y):
                if temp is not self.ws.height_map_array[x][y]:
                    return False
        return True

    def set_size(w,h, ax=None):
        """ w, h: width, height in inches """
        if not ax: ax=plt.gca()
        l = ax.figure.subplotpars.left
        r = ax.figure.subplotpars.right
        t = ax.figure.subplotpars.top
        b = ax.figure.subplotpars.bottom
        figw = float(w)/(r-l)
        figh = float(h)/(t-b)
        ax.figure.set_size_inches(figw, figh)

    def first_fit_algorithm(self, parcel):
        ws_x = self.ws.workspace_size[0]
        ws_y = self.ws.workspace_size[1]


        for y in range(ws_y):
            for x in range(ws_x):
                z = self.ws.height_map_array[x][y] #Get z for (x,y) coordinate
                print("x: ", x, " | y:", y, " | z:", z) 
                in_range = self.parcel_in_range((x,y,z), parcel)
                print("In Range: ", in_range)
                if in_range is True:
                    supported = self.bottom_supported((x,y),parcel)
                    print("Supported: ", supported)
                    if supported is True:
                        print("(x,y,z): ", (x,y,z))
                        #Publish to a ros topic
                        self.packing_pub(x, y, z, parcel.size[0], parcel.size[1], parcel.size[2])
                        #Update height map
                        #Add parcel to height map, by changing each pixel (x,y) to the height
                        for i in range(x, x + parcel.size[0]): #x
                            for j in range(y, y + parcel.size[1]): #y
                                #print("sizes: ", parcel.size[0], " ", parcel.size[1])
                                #print("wtf", i, " ", j)
                                self.ws.height_map_array[i][j] = parcel.size[2] + z
                        return (x,y,z) #Return x,y coordinate for parcel
            
        return False

    #Goes through all the different rotations for the package
    def start_first_fit(self, parcel):
        if self.first_fit_algorithm(parcel) == False:
            parcel.rotate_parcel('z')
            if self.first_fit_algorithm(parcel) == False:       
                parcel.rotate_parcel('y')
                if self.first_fit_algorithm(parcel) == False:       
                    parcel.rotate_parcel('z')
                    if self.first_fit_algorithm(parcel) == False:       
                        parcel.rotate_parcel('y')
                        if self.first_fit_algorithm(parcel) == False:       
                            parcel.rotate_parcel('z')
                            if self.first_fit_algorithm(parcel) == False:       
                                print("Parcel cannot be packed into the roller cage")

    def packing_pub(self, pos_x, pos_y, pos_z, size_x, size_y, size_z):
        msg = Packing_info()

        msg.size = Point(size_x, size_y, size_z)
        msg.pos = Point(pos_x, pos_y, pos_z)


        print(msg)

        self.pub.publish(msg)
        print("Publishing to /packing_info")


def main():
    rospy.init_node('first_fit_algorithm', anonymous=True)

    ws = workspace(20, 20, 20) #Create a new instance of the workspace class

    ff = first_fit(ws) #Create a new instance of the first fit class

    #p = parcel((0,0,0), (1,1,1))

    #spin
    rospy.spin()

if __name__ == '__main__':
    main()