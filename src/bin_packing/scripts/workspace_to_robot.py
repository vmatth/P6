#!/usr/bin/env python
from lib2to3.pytree import convert
import rospy
from bin_packing.msg import Packing_info
from bin_packing.msg import Workspace #workspace msg
from geometry_msgs.msg import Point
import math
import numpy as np
import cv2

#Converts a position given in the workspace frame to a position on the robot frame
class converter():
    def __init__(self):
        print("workspace to robot node")
        rospy.init_node('workspace_to_robot_converter', anonymous=True)
        rospy.Subscriber("/workspace/add_parcel", Packing_info, self.convert_frames)   
        self.pub = rospy.Publisher('/robot/pick_place', Packing_info, queue_size=10)
        rospy.Subscriber("/workspace/info", Workspace, self.workspace_callback)
        self.cage_x_displacement = 0
        self.cage_y_displacement = 0
        self.cage_z_displacement = 0



    def workspace_callback(self, data):
        print("Workspace callbak in converter node")
        #How much the roller cage frame is displaced from the robot's frame [m]

        #Convert roller_cage frame to robot frame
        #robot frame             roller cage frame
        #       ^                       ^
        #      x|                       | y
        #    y  |                       |    x
        # <-----o                       o----->
        self.cage_x_displacement = data.corner_position.x
        self.cage_y_displacement = data.corner_position.y
        self.cage_z_displacement = data.corner_position.z

        print("Workspace corner relative to robot frame: ", Point(self.cage_x_displacement, self.cage_y_displacement, self.cage_z_displacement))

    def matrix_from_rtvec(self, rvec, tvec):
        (R, jac) = cv2.Rodrigues(rvec) # ignore the jacobian
        M = np.eye(4)
        M[0:3, 0:3] = R
        M[0:3, 3] = tvec.squeeze() # 1-D vector, row vector, column vector, whatever
        return M


    #Converts start_pos and end_pos from the topic /workspace/add_parcel to respect the robot frame.
    def convert_frames(self, data):
        rospy.loginfo(rospy.get_caller_id() + "New parcel to convert frames %s", data)
        converted_data = Packing_info()
        converted_data.angle = data.angle
        converted_data.picking_side = data.picking_side
        converted_data.parcel_rotation = data.parcel_rotation
        converted_data.actual_size = data.actual_size
        converted_data.rounded_size = data.rounded_size
        converted_data.non_rotated_size = data.non_rotated_size

        ##ALL DISTANCES ARE IN METRES

        #Convert camera frame to robot frame
        #robot frame             camera frame
        #       ^                       ^
        #      x|                       | x
        #    y  |                       |    y
        # <-----o                       o----->

        #How much the camera frame is displaced from the robot's frame [m] 
        cam_x_displacement = 0.042
        cam_y_displacement = -0.528
        cam_z_displacement = 0
        #xyz="0.0858675 -0.480904 1.13782" Danilidis
        # 0.061482 -0.485282 0.984972 ParkBryan
        # 0.11 -0.269585 0.455615 Tsai

        #Transformation matrix using rpy and xyz
        rvec = np.array([3.13, 0.105, -0.09])
        tvec = np.array([0.085, -0.48, 1.13])
        T = self.matrix_from_rtvec(rvec, tvec)
        #print("T: ", T)

        #Transformation frame using values given from MATLAB
        T = np.array([[0.5010, 0.7962, -0.3392, 0.0858675], [0.8294, -0.3298, 0.4509, -0.470904], [0.2472, -0.5072, -0.8256, 1.13782], [0,0,0,1]])
        print("T:", T)

        #Vector of the point found in respect to the camera frame
        p = np.array([data.start_pos.x/100, data.start_pos.y/100, data.start_pos.z/100, 1]) 

        print("p: ", p)

        result = np.matmul(T, p)
        print("result: ", result)

        converted_data.start_pos.x = result[0]
        converted_data.start_pos.y = result[1]
        converted_data.start_pos.z = data.start_pos.z/100

        #OLD CONVERTER
        # #Calculate the point with respect to the robot's frame (The incoming data is in cm so we convert to m)
        # converted_data.start_pos.x = (data.start_pos.x/100) + cam_x_displacement
        # converted_data.start_pos.y = (data.start_pos.y/100) * -1 + cam_y_displacement
        # converted_data.start_pos.z = (data.start_pos.z/100) + cam_z_displacement

        #Data from the packing algorithm is specified from the corner closest to (0,0) and not the center point which the robot uses.
        #The next three lines converts it to a center point. #We convert to m again
        data.end_pos.x = data.end_pos.x/100 + (data.actual_size.x/100/2)
        data.end_pos.y = data.end_pos.y/100 + (data.actual_size.y/100/2)
        data.end_pos.z = data.end_pos.z/100 + (data.actual_size.z/100)

        #Convert roller_cage frame to robot frame
        #robot frame             roller cage frame
        #       ^                       ^
        #      x|                       | y
        #    y  |                       |    x
        # <-----o                       o----->
        converted_data.end_pos.x = data.end_pos.y + self.cage_x_displacement
        converted_data.end_pos.y = data.end_pos.x * -1 + self.cage_y_displacement
        converted_data.end_pos.z = data.end_pos.z + self.cage_z_displacement
        

        rospy.loginfo(rospy.get_caller_id() + "Converted frames %s", converted_data)

        self.pub.publish(converted_data)


def main():
    #lav en instance af klassen
    c = converter()

    rospy.spin()


if __name__ == '__main__':
    main()

