#!/usr/bin/env python
from lib2to3.pytree import convert
import rospy
from bin_packing.msg import Packing_info

#Converts a position given in the workspace frame to a position on the robot frame


class converter():
    def __init__(self):
        rospy.init_node('workspace_to_robot_converter', anonymous=True)
        rospy.Subscriber("/workspace/add_parcel", Packing_info, self.convert_frames)   
        self.pub = rospy.Publisher('/robot/pick_place', Packing_info, queue_size=10)

    #Converts start_pos and end_pos from the topic /workspace/add_parcel to respect the robot frame.
    def convert_frames(self, data):
        rospy.loginfo(rospy.get_caller_id() + "New parcel to convert frames %s", data)
        converted_data = Packing_info()
        converted_data.angle = data.angle
        converted_data.picking_side = data.picking_side
        converted_data.parcel_rotation = data.parcel_rotation
        converted_data.size = data.size

        ##ALL DISTANCES ARE IN METRES

        #Convert camera frame to robot frame
        #robot frame             camera frame
        #       ^                       ^
        #      x|                       | x
        #    y  |                       |    y
        # <-----o                       o----->

        #How much the camera frame is displaced from the robot's frame
        cam_x_displacement = 7
        cam_y_displacement = 0
        cam_z_displacement = 2
        #Calculate the point with respect to the robot's frame
        converted_data.start_pos.x = data.start_pos.x + cam_x_displacement
        converted_data.start_pos.y = data.start_pos.y * -1 + cam_y_displacement
        converted_data.start_pos.z = data.start_pos.z * -1 + cam_z_displacement

        #Data from the packing algorithm is specified from the corner closest to (0,0) and not the center point which the robot uses.
        #The next three lines converts it to a center point.
        data.end_pos.x = data.end_pos.x + (data.size.x/2)
        data.end_pos.y = data.end_pos.y + (data.size.y/2)
        data.end_pos.z = data.end_pos.z + data.size.z

        #Convert roller_cage frame to robot frame
        #robot frame             roller cage frame
        #       ^                       ^
        #      x|                       | y
        #    y  |                       |    x
        # <-----o                       o----->
        #How much the roller cage frame is displaced from the robot's frame
        cage_x_displacement = -0.36
        cage_y_displacement = 0.94
        cage_z_displacement = -0.18
        converted_data.end_pos.x = data.end_pos.y + cage_x_displacement
        converted_data.end_pos.y = data.end_pos.x * -1 + cage_y_displacement
        converted_data.end_pos.z = data.end_pos.z + cage_z_displacement
        

        rospy.loginfo(rospy.get_caller_id() + "Converted frames %s", converted_data)

        self.pub.publish(converted_data)


def main():
    #lav en instance af klassen
    c = converter()

    rospy.spin()


if __name__ == '__main__':
    main()

