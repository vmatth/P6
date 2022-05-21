#!/usr/bin/env python
from time import sleep
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
import random

def packing_pub(size):
    msg = Parcel()

    msg.size = size

    print(msg)

    pub.publish(msg)
    print("Publishing to /vision/parcel")

def add_parcel(data):
    # contraints:
    # min: 14x9x8 cm ==> 14+((9+8)*2) = 48
    # max: length 150 cm 
    # max: length + circumfence 300 cm     150+(37,5+37,5)*2 = 300
    rospy.sleep(0.1)
    ###############################
    sendle_parcel_generator()
    ##Change parcel randomizer here
    ##Options are dao() or postnord()
    #dao()
    #tiny()
    ###############################

def sendle_parcel_generator():
    list_of_parcels = [Point(10, 10, 10), Point(10,10,15), Point(10,10,20), Point(13,13,13), Point(15,15,10), Point(20,15,8), Point(23,15,8)]
    element = random.randrange(0,7)
    packing_pub(list_of_parcels[element])

def tiny():
    x = random.randrange(8, 12)
    y = random.randrange(8, 12)
    z = random.randrange(8, 12)
    packing_pub(Point(x,y,z))
    
def dao():
    x = random.randrange(15, 80)
    print("x: ", x)
    circumfence = random.randrange(48, 240-x)
    print("circumfence: ", circumfence)
    y = random.randrange(10,(circumfence-8)/2)
    print("y: ", y)
    z = (circumfence - y)/2
    print("z: ", z)
    packing_pub(Point(x,y,z))

def postnord():
    x = random.randrange(14, 150)
    print("x: ", x)
    circumfence = random.randrange(48, 300-x)
    print("circumfence: ", circumfence)
    y = random.randrange(9,(circumfence-8)/2)
    print("y: ", y)
    z = (circumfence - y)/2
    print("z: ", z)
    packing_pub(Point(x,y,z))


def reset_workspace(data):
    print("Workspace has been reset")
    rospy.sleep(2)
    global seed
    seed = seed + 1
    random.seed(seed)
    add_parcel(None)

seed = 1

pub = rospy.Publisher('/vision/parcel', Parcel, queue_size=10)
def main():
    random.seed(seed)
    rospy.init_node('send_parcels', anonymous=True)
    sub = rospy.Subscriber("/workspace/add_parcel", Packing_info, add_parcel) #Runs in a loop to keep publishing new parcels
    sub = rospy.Subscriber("/workspace/remove_parcels", Workspace, reset_workspace) #This is called when the workspace is reset. This function increases the random seed.
    pub = rospy.Publisher('/vision/parcel', Parcel, queue_size=10)
    rospy.sleep(2)
    add_parcel(None)
    rospy.spin()

if __name__ == '__main__':
    main()