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
from bin_packing.msg import Workspace #workspace msg
from bin_packing.convertTo2DArray import convertTo2DArray #convert function


class floor_building:
    def __init__(self):
        print("init floor building")

        self.workspace_size = Point(0, 0, 0)
        self.height_map = [[]]
        
        self.pub = rospy.Publisher('/workspace/add_parcel', Packing_info, queue_size=10)
        
        #self.pub = rospy.Publisher('/packing_info', Packing_info, queue_size=10)
        
        rospy.Subscriber("/parcel_info", Parcel, self.parcel_callback)
        rospy.Subscriber("/workspace/info", Workspace, self.workspace_callback)
    
    
    def workspace_callback(self, data):
        print("/workspace/info callback")
        #rospy.loginfo(rospy.get_caller_id() + "Receiving data from /workspace/info %s", data)
        self.workspace_size = data.size
        self.height_map = convertTo2DArray(data.height_map, False)
        #print("new height_map_ ", self.height_map)

    
    def parcel_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Receiving data from /parcel_info %s", data)
        p = parcel(Point(0,0,0), data.size)
        self.start_floor_building(p)

    def parcel_in_range(self, position, parcel):
            size_x = parcel.size.x
            size_y = parcel.size.y
            size_z = parcel.size.z
            pos_x = position.x
            pos_y = position.y
            pos_z = position.z
            #print("Checking if parcel is in range. Size: ", parcel.size, " | Pos: ", position)

            #print("ws size_ ", self.ws.workspace_size)
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

    def bottom_supported(self, position, parcel):
            size_x = int(parcel.size.x)
            size_y = int(parcel.size.y)
            pos_x = int(position.x)
            pos_y = int(position.y)

            corner_pixels = ((pos_x, pos_y),(pos_x+size_x-1,pos_y),(pos_x+size_x-1,pos_y+size_y-1),(pos_x,pos_y+size_y-1))
            supported_pixels = 0.0
            counter = 0
            temp = self.height_map[pos_x][pos_y]
            for x in range(pos_x, pos_x + size_x):
                for y in range(pos_y, pos_y + size_y):
                    if temp == self.height_map[x][y]: #Check if (x,y) coordinate is supported
                            supported_pixels += 1
                            #check if x, y is one of the corner pixels
                            if (x,y) in corner_pixels:
                                counter += 1
            #print("supported pixel: ", supported_pixels)
            total_parcel_pixels = float(size_x) * float(size_y)
            #print("total: ", total_parcel_pixels)
            bottom_area_supported = supported_pixels/total_parcel_pixels * 100
            #print("Area supported: ", int(bottom_area_supported), "%")
            if bottom_area_supported >= 95:
                #print("95 percent stability requirement")
                return True
            elif counter >= 3 and bottom_area_supported >= 80:
                #print("80 percent and 3 corners stability requirement")
                return True
            elif counter == 4 and bottom_area_supported >= 60:
                #print("60 percent and 4 corners stability requirement")
                return True 
            else:
                return False
            

    def floor_building_algorithm(self, _parcel):
        ws_x = int(self.workspace_size.x)
        ws_y = int(self.workspace_size.y)
        xyzlist = []     
        print("floor building algorithm")

        r = 0 #Times rotated
        original_parcel = parcel(Point(0,0,0), _parcel.size)
        h = _parcel.size.z

        for y in range(ws_y):
            for x in range(ws_x):
                z = self.height_map[x][y] #Get z for (x,y) coordinate
                print("In range func | (xy): ", (x,y), "z: ", z)
                # print("x: ", x, " | y:", y, " | z:", z) 
                in_range = self.parcel_in_range(Point(x,y,z), _parcel)
                #print("In Range: ", in_range)
                if in_range is True:
                    supported = self.bottom_supported(Point(x,y,0),_parcel)
                    #print("Supported: ", supported)
                    if supported is True:
                        #print("(x,y,z): ", (x,y,z))
                        xyzlist.append((x,y,z, r, h))
                        #print("list: ", xyzlist)

        _parcel.rotate_parcel('z')
        r = r + 1
        h = _parcel.size.z
        for y in range(ws_y):
            for x in range(ws_x):
                z = self.height_map[x][y] #Get z for (x,y) coordinate
                # print("x: ", x, " | y:", y, " | z:", z) 
                in_range = self.parcel_in_range(Point(x,y,z), _parcel)
                #print("In Range: ", in_range)
                if in_range is True:
                    supported = self.bottom_supported(Point(x,y,0),_parcel)
                    #print("Supported: ", supported)
                    if supported is True:
                        #print("(x,y,z): ", (x,y,z))
                        xyzlist.append((x,y,z, r, h))
                        #print("list: ", xyzlist)

        _parcel.rotate_parcel('y')
        r = r + 1
        h = _parcel.size.z
        for y in range(ws_y):
            for x in range(ws_x):
                z = self.height_map[x][y] #Get z for (x,y) coordinate
                # print("x: ", x, " | y:", y, " | z:", z) 
                in_range = self.parcel_in_range(Point(x,y,z), _parcel)
                #print("In Range: ", in_range)
                if in_range is True:
                    supported = self.bottom_supported(Point(x,y,0),_parcel)
                    #print("Supported: ", supported)
                    if supported is True:
                        #print("(x,y,z): ", (x,y,z))
                        xyzlist.append((x,y,z, r, h))
                        #print("list: ", xyzlist)
        _parcel.rotate_parcel('z')
        r = r + 1
        h = _parcel.size.z
        for y in range(ws_y):
            for x in range(ws_x):
                z = self.height_map[x][y] #Get z for (x,y) coordinate
                # print("x: ", x, " | y:", y, " | z:", z) 
                in_range = self.parcel_in_range(Point(x,y,z), _parcel)
                #print("In Range: ", in_range)
                if in_range is True:
                    supported = self.bottom_supported(Point(x,y,0),_parcel)
                    #print("Supported: ", supported)
                    if supported is True:
                        #print("(x,y,z): ", (x,y,z))
                        xyzlist.append((x,y,z, r, h))
                        #print("list: ", xyzlist)
        _parcel.rotate_parcel('y')
        r = r + 1
        h = _parcel.size.z
        for y in range(ws_y):
            for x in range(ws_x):
                z = self.height_map[x][y] #Get z for (x,y) coordinate
                # print("x: ", x, " | y:", y, " | z:", z) 
                in_range = self.parcel_in_range(Point(x,y,z), _parcel)
                #print("In Range: ", in_range)
                if in_range is True:
                    supported = self.bottom_supported(Point(x,y,0),_parcel)
                    #print("Supported: ", supported)
                    if supported is True:
                        #print("(x,y,z): ", (x,y,z))
                        xyzlist.append((x,y,z, r, h))
                        #print("list: ", xyzlist)
        _parcel.rotate_parcel('z')
        r = r + 1
        h = _parcel.size.z
        for y in range(ws_y):
            for x in range(ws_x):
                z = self.height_map[x][y] #Get z for (x,y) coordinate
                # print("x: ", x, " | y:", y, " | z:", z) 
                in_range = self.parcel_in_range(Point(x,y,z), _parcel)
                #print("In Range: ", in_range)
                if in_range is True:
                    supported = self.bottom_supported(Point(x,y,0),_parcel)
                    #print("Supported: ", supported)
                    if supported is True:
                        #print("(x,y,z): ", (x,y,z))
                        xyzlist.append((x,y,z, r, h))
                        #print("list: ", xyzlist)
        # print("r", r)
        # print("h", h)

        #print("xyzlist: ", xyzlist)


        if len(xyzlist) > 0:
            temp = xyzlist[0]
            for i in range(len(xyzlist)):
                #print("temppp, ", temp)
                #print("xyzlist", xyzlist[i])
                #print("temp[4]", temp[4], "xyzlist[4]", xyzlist[i][4]) 
                if xyzlist[i][2] < temp[2]:
                    temp = xyzlist[i]
                    #print("new temp ", temp)
                elif xyzlist[i][4] < temp[4] and xyzlist[i][2] <= temp[2]:
                    temp = xyzlist[i]
                    #print("new temp ", temp)
            #print("parcel: ", _parcel.size.x, _parcel.size.y, _parcel.size.z)            
            #print("temp: ", temp)
            #publish x,y,z coordinates
            #Fundet den laveste z-coordinate
            if temp[3] == 0:
                print("No rotation")
            elif temp[3] == 1:
                print("Rotated parcel 1 time")
                original_parcel.rotate_parcel('z')
            elif temp[3] == 2:
                print("Rotated parcel 2 times")
                original_parcel.rotate_parcel('z')   
                original_parcel.rotate_parcel('y')    
            elif temp[3] == 3:
                print("Rotated parcel 3 times")
                original_parcel.rotate_parcel('z')   
                original_parcel.rotate_parcel('y')  
                original_parcel.rotate_parcel('z') 
            elif temp[3] == 4:
                print("Rotated parcel 4 times")
                original_parcel.rotate_parcel('z')   
                original_parcel.rotate_parcel('y')  
                original_parcel.rotate_parcel('z')
                original_parcel.rotate_parcel('y')
            elif temp[3] == 5:
                print("Rotated parcel 5 times")
                original_parcel.rotate_parcel('z')   
                original_parcel.rotate_parcel('y')  
                original_parcel.rotate_parcel('z')
                original_parcel.rotate_parcel('y')
                original_parcel.rotate_parcel('z')

                
            self.packing_pub(temp[0], temp[1], temp[2], original_parcel.size.x, original_parcel.size.y, original_parcel.size.z)

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
                                rospy.sleep(999)
        
    def packing_pub(self, pos_x, pos_y, pos_z, size_x, size_y, size_z):
        msg = Packing_info()

        msg.size = Point(size_x, size_y, size_z)
        msg.pos = Point(pos_x, pos_y, pos_z)


        print(msg)

        self.pub.publish(msg)
        print("Publishing to /packing_info")



def main():
    rospy.init_node('floor_building_algorithm', anonymous=True)
    fb =  floor_building() #Create a new instance of the first fit class
    #spin
    rospy.spin()


if __name__ == '__main__':

    main()

