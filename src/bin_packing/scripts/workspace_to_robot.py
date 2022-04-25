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
        # o----->                ^
        # |    x                 | x
        # | y                    |    y
        # v                      o----->

        #How much the camera frame is displaced from the robot's frame
        cam_x_displacement = 7
        cam_y_displacement = 0
        cam_z_displacement = 2
        #Calculate the point with respect to the robot's frame
        converted_data.start_pos.x = data.start_pos.y + cam_x_displacement
        converted_data.start_pos.y = data.start_pos.x * -1 + cam_y_displacement
        converted_data.start_pos.z = data.start_pos.z * -1 + cam_z_displacement

        #Convert roller_cage frame to robot frame
        #robot frame             roller cage frame
        # o----->                ^
        # |    x                 | y
        # | y                    |    x
        # v                      o----->
        #How much the roller cage frame is displaced from the robot's frame
        cage_x_displacement = -8
        cage_y_displacement = 4
        cage_z_displacement = -2
        converted_data.end_pos.x = data.end_pos.x + cage_x_displacement
        converted_data.end_pos.y = data.end_pos.y * -1 + cage_y_displacement
        converted_data.end_pos.z = data.end_pos.z + cage_z_displacement

        rospy.loginfo(rospy.get_caller_id() + "Converted frames %s", converted_data)

        self.pub.publish(converted_data)


def main():
    #lav en instance af klassen
    c = converter()

    rospy.spin()


if __name__ == '__main__':
    main()

