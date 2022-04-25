#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point #point type

#Class for a new individual parcel. Contains information about the parcel's position and parcel's size.
class parcel:
    def __init__(self, _position, _size, _start_position):
        self.size = _size
        self.position = _position #The goal position where the parcel will be placed in the roller cage (in respect to the roller cage frame)
        self.start_position = _start_position #The position where the parcel is placed (in respect to the camera frame)

        #Rotates a parcel around an axis: 'x', 'y', or 'z'.
    def rotate_parcel(self, axis):
        x = self.size.x
        y = self.size.y
        z = self.size.z
        #print("Values before rotating:")
        #print(self.size)

        # print("Rotating around: ", axis)

        if axis == 'x':
            self.size = Point(x, z, y)
        elif axis == 'y':
            self.size = Point(z, y, x)
        elif axis == 'z':
            self.size = Point(y, x ,z)

    
        #print("Values after rotating:")
        #print(self.size)
        return parcel
