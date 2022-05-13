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

# def generate_parcels(pub):
    

#     for i in range(4): 
#         rospy.sleep(1.5)


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
    ##Change parcel randomizer here
    ##Options are dao() or postnord()
    dao()
    ###############################

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

pub = rospy.Publisher('/vision/parcel', Parcel, queue_size=10)
def main():
    random.seed(15)
    rospy.init_node('send_parcels', anonymous=True)
    sub = rospy.Subscriber("/workspace/add_parcel", Packing_info, add_parcel)
    pub = rospy.Publisher('/vision/parcel', Parcel, queue_size=10)
    rospy.sleep(2)
    add_parcel(None)
    #generate_parcels(pub)
    rospy.spin()

if __name__ == '__main__':
    main()