#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point #point type
import math

#Class for a new individual parcel. Contains information about the parcel's position and parcel's size.
class parcel:
    def __init__(self, _position, _actual_size, _start_position, _angle):
        self.position = _position #The goal position where the parcel will be placed in the roller cage (in respect to the roller cage frame)
        self.start_position = _start_position #The position where the parcel is placed (in respect to the camera frame)
        self.angle = _angle #Angle from the camera. 
        self.rounded_size = Point(math.ceil(_actual_size.x) + 1, math.ceil(_actual_size.y) + 1, math.ceil(_actual_size.z))
        self.actual_size = _actual_size

        #Rotates a parcel around an axis: 'x', 'y', or 'z'.
    def rotate_parcel(self, axis):
        x = self.actual_size.x
        y = self.actual_size.y
        z = self.actual_size.z
        _x = self.rounded_size.x
        _y = self.rounded_size.y
        _z = self.rounded_size.z
        #print("Values before rotating:")
        #print(self.size)

        # print("Rotating around: ", axis)

        if axis == 'x':
            self.actual_size = Point(x, z, y)
            self.rounded_size = Point(_x, _z, _y)
        elif axis == 'y':
            self.actual_size = Point(z, y, x)
            self.rounded_size = Point(_z, _y, _x)
        elif axis == 'z':
            self.actual_size = Point(y, x , z)
            self.rounded_size = Point(_y, _x, _z)

    
        #print("Values after rotating:")
        #print(self.size)
        return parcel
