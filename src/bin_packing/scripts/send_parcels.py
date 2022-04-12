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
    print("Publishing to /parcel_info")

def add_parcel(data):
    rospy.loginfo(rospy.get_caller_id() + "Workspace received new parcel %s", data)
    rospy.sleep(1)
    packing_pub(Point(random.randrange(3,7),random.randrange(3,7), random.randrange(3,7)))

pub = rospy.Publisher('/parcel_info', Parcel, queue_size=10)
def main():
    random.seed(8)
    rospy.init_node('send_parcels', anonymous=True)
    sub = rospy.Subscriber("/workspace/add_parcel", Packing_info, add_parcel)
    pub = rospy.Publisher('/parcel_info', Parcel, queue_size=10)
    rospy.sleep(2)
    packing_pub(Point(random.randrange(3,7),random.randrange(3,7), random.randrange(3,7)))
    #generate_parcels(pub)
    rospy.spin()

if __name__ == '__main__':
    main()