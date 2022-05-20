#!/usr/bin/env python
from lib2to3.pytree import convert
import rospy
from bin_packing.msg import Packing_info
from bin_packing.msg import Workspace #workspace msg
from geometry_msgs.msg import Point
import math

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
        cam_x_displacement = 0.04
        cam_y_displacement = -0.530
        cam_z_displacement = 0
        #xyz="0.0858675 -0.480904 1.13782" Danilidis
        # 0.061482 -0.485282 0.984972 ParkBryan
        # 0.11 -0.269585 0.455615 Tsai


        #Calculate the point with respect to the robot's frame (The incoming data is in cm so we convert to m)
        converted_data.start_pos.x = (data.start_pos.x/100) + cam_x_displacement
        converted_data.start_pos.y = (data.start_pos.y/100) * -1 + cam_y_displacement
        converted_data.start_pos.z = (data.start_pos.z/100) + cam_z_displacement

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

