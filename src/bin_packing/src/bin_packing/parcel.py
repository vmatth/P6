#!/usr/bin/env python
import rospy

#Class for a new individual parcel. Contains information about the parcel's position and parcel's size.
class parcel:
    def __init__(self, _position, _size):
        self.size = _size
        self.position = _position

        #Rotates a parcel around an axis: 'x', 'y', or 'z'.
    def rotate_parcel(self, axis):
        x = self.size[0]
        y = self.size[1]
        z = self.size[2]
        print("Values before rotating:")
        print(self.size)

        print("Rotating around: ", axis)

        if axis == 'x':
            self.size = (x, z, y)
        elif axis == 'y':
            self.size = (z, y, x)
        elif axis == 'z':
            self.size = (y, x ,z)

    
        print("Values after rotating:")
        print(self.size)
        return parcel
