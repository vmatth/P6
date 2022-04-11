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

def generate_parcels(pub):
    random.seed(69) 

    for i in range(300):

        packing_pub(Point(random.randrange(18,45),random.randrange(18,45), random.randrange(18,45)), pub)
        rospy.sleep(0.3)

def packing_pub(size, pub):
    msg = Parcel()

    msg.size = size

    print(msg)

    pub.publish(msg)
    print("Publishing to /parcel_info")



def main():
    rospy.init_node('send_parcels', anonymous=True)
    pub = rospy.Publisher('/parcel_info', Parcel, queue_size=10)
    rospy.sleep(2)
    generate_parcels(pub)


if __name__ == '__main__':
    main()