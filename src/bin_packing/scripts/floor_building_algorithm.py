#!/usr/bin/env python
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

class workspace:

    #Init function for the packing setup. Creates the workspace & height map plots. Workspace sizes are in [cm]. 
    def __init__(self, x, y, z):
        #rospy.Subscriber("/parcel_info", Parcel , self.parcel_callback)

        self.parcels = [] #Stores all of the parcels in the workspace

        self.workspace_size = (x, y, z) #Save the workspace size in a variable

        self.height_map_array = [[0 for i in range(x)] for j in range(y)] 


# packing_info, output from algorithm: position and size
# parcel_info, width, height, depth


class floor_building:
    def __init__(self, _ws):
        print("init floor building")
        print("workspace size:", _ws.workspace_size)
        self.ws = _ws
        self.pub = rospy.Publisher('/packing_info', Packing_info, queue_size=10)
        rospy.Subscriber("/parcel_info", Parcel, self.parcel_callback)

    def parcel_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        p = parcel((0,0,0), (int(data.width), int(data.height), int(data.depth)))
        self.start_floor_building(p)

    def parcel_in_range(self, position, parcel):
            size_x = parcel.size[0]
            size_y = parcel.size[1]
            size_z = parcel.size[2]
            pos_x = position[0]
            pos_y = position[1]
            pos_z = position[2]
            #print("Checking if parcel is in range. Size: ", parcel.size, " | Pos: ", position)

            #print("ws size_ ", self.ws.workspace_size)
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

    def bottom_supported(self, position, parcel):
            size_x = int(parcel.size[0])
            size_y = int(parcel.size[1])
            pos_x = int(position[0])
            pos_y = int(position[1])

            #print("Checking if parcel is SUPPORTED. Size: ", parcel.size, " | Pos: ", position)

            temp = self.ws.height_map_array[pos_x][pos_y]
            for x in range(pos_x, pos_x + size_x):
                for y in range(pos_y, pos_y + size_y):
                    if temp is not self.ws.height_map_array[x][y]:
                        return False
            return True

    def floor_building_algorithm(self, _parcel):
        #run igennem alle x,y vaerdier
            #kan pakken placeres paa denne koordinat?
                #hvis ja: gem (x,y,z)
        #sammenlign alle hoejder
        #select den laveste koordinat
        #publish den laveste (x,y,z)
        ws_x = self.ws.workspace_size[0]
        ws_y = self.ws.workspace_size[1]
        xyzlist = [] 

        temp_parcel = _parcel
        print("og parcel", parcel)
        print("temp parcel", temp_parcel)

        for y in range(ws_y):
            for x in range(ws_x):
                z = self.ws.height_map_array[x][y] #Get z for (x,y) coordinate
               # print("x: ", x, " | y:", y, " | z:", z) 
                in_range = self.parcel_in_range((x,y,z), _parcel)
                #print("In Range: ", in_range)
                if in_range is True:
                    supported = self.bottom_supported((x,y),_parcel)
                    #print("Supported: ", supported)
                    if supported is True:
                        #print("(x,y,z): ", (x,y,z))
                        xyzlist.append((x,y,z))
                        #print("list: ", xyzlist)
    
        #rotate z her
        temp_parcel.rotate_parcel('z')
        
        # for y in range(ws_y):
        #     for x in range(ws_x):
        #         z = self.ws.height_map_array[x][y] #Get z for (x,y) coordinate
        #         #print("x: ", x, " | y:", y, " | z:", z) 
        #         in_range = self.parcel_in_range((x,y,z), temp_parcel)
        #        # print("In Range: ", in_range)
        #         if in_range is True:
        #             supported = self.bottom_supported((x,y),temp_parcel)
        #            # print("Supported: ", supported)
        #             if supported is True:
        #                 #print("(x,y,z): ", (x,y,z))
        #                 xyzlist.append((x,y,z))
        #                 #print("list: ", xyzlist)

        #kor samme for loop lmao

        #rotate igen

        #korr samme for loop

        if len(xyzlist) > 0:
            temp = xyzlist[0]
            for i in range(len(xyzlist)):
                #print("temppp, ", temp)
                #print("xyzlist", xyzlist[i])
                if xyzlist[i][2] < temp[2]:
                    temp = xyzlist[i]
            print("parcel: ", _parcel.size.x, _parcel.size.y, _parcel.size.z)            
            print("temp: ", temp)
            #publish x,y,z coordinates
            self.packing_pub(temp[0], temp[1], temp[2], _parcel.size.x, _parcel.size.y, _parcel.size.z)
            for i in range(temp[0], temp[0] + _parcel.size.x): #x
                for j in range(temp[1], temp[1] + _parcel.size.y): #y
                    print("i, j: ", i, j)
                    #print("sizes: ", parcel.size[0], " ", parcel.size[1])
                    #print("wtf", i, " ", j)
                    self.ws.height_map_array[i][j] = _parcel.size.z + temp[2]

        elif len(xyzlist) <= 0:
            return False


    def start_floor_building(self, parcel):
        if self.floor_building_algorithm(parcel) == False:
            parcel.rotate_parcel('z')
            if self.floor_building_algorithm(parcel) == False:
                parcel.rotate_parcel('y')
                if self.floor_building_algorithm(parcel) == False:
                    parcel.rotate_parcel('z')
                    if self.floor_building_algorithm(parcel) == False:
                        parcel.rotate_parcel('y')
                        if self.floor_building_algorithm(parcel) == False:
                            parcel.rotate_parcel('z')
                            if self.floor_building_algorithm(parcel) == False:
                                print("Parcel cannot be packed into the roller cage")
        
    def packing_pub(self, pos_x, pos_y, pos_z, size_x, size_y, size_z):
        msg = Packing_info()

        msg.size = Point(size_x, size_y, size_z)
        msg.pos = Point(pos_x, pos_y, pos_z)


        print(msg)

        self.pub.publish(msg)
        print("Publishing to /packing_info")



def main():
    rospy.init_node('floor_building_algorithm', anonymous=True)
    ws = workspace(20, 20, 20) #Create a new instance of the workspace class

    fb =  floor_building(ws) #Create a new instance of the first fit class

    # p = parcel((0,0,0), (1,1,1))

    # fb.floor_building_algorithm(p)

    #spin
    rospy.spin()


if __name__ == '__main__':

    main()

