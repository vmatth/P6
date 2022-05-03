#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose 
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion 
from read_camera.msg import Parcel #Parcel msg
from scipy.spatial.transform import Rotation
from bin_packing.msg import Packing_info
from bin_packing.msg import Workspace #workspace msg
from ur_msgs.srv import SetIO


#class here
class mover:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_mover', anonymous=True)
        self.sim = rospy.get_param("~sim", False)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.parcel_goal = geometry_msgs.msg.Pose()

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        self.planning_frame = self.group.get_planning_frame()
        self.eef_link = self.group.get_end_effector_link()

        self.parcels_packed = 0

        rospy.sleep(2)

        self.add_environment()
        self.clear_workspace()
        
        rospy.Subscriber("/robot/pick_place", Packing_info, self.add_parcel)
        print("Sim ", self.sim)
        self.go_to_pose("idle")
        #self.print_info()

    def print_info(self):
        print("end effector: ", self.eef_link)
        
        print("planning frame: ", self.planning_frame)
        print("Robot Groups: ", self.robot.get_group_names())
        print("Robot State: ", self.robot.get_current_state())
        print("Robot Pose: ", self.group.get_current_pose())
        
    def add_environment(self):
        print("Adding environment")
        small_table_pose = geometry_msgs.msg.PoseStamped()
        small_table_pose.header.frame_id = "base_link"
        small_table_pose.pose.orientation.w = 1.0
        small_table_pose.pose.position.x = 0
        small_table_pose.pose.position.y = -0.065
        small_table_pose.pose.position.z = -0.014
        small_table_name = "small_table"
        self.scene.add_box(small_table_name, small_table_pose, size=(0.39, 0.39, 0.018))

        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "base_link"
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = 0
        table_pose.pose.position.y = -0.31
        table_pose.pose.position.z = -0.055
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=(0.41, 0.93, 0.084))
        
        backboard_pose = geometry_msgs.msg.PoseStamped()
        backboard_pose.header.frame_id = "base_link"
        backboard_pose.pose.orientation.w = 1.0
        backboard_pose.pose.position.x = 0.0
        backboard_pose.pose.position.y = -0.80   
        backboard_pose.pose.position.z = 0.445
        backboard_name = "backboard"
        self.scene.add_box(backboard_name, backboard_pose, size=(0.41, 0.05, 1.06))

        workspace_pose = geometry_msgs.msg.PoseStamped()
        workspace_pose.header.frame_id = "base_link"
        workspace_pose.pose.orientation.w = 1.0
        workspace_pose.pose.position.x = 0.0
        workspace_pose.pose.position.y = 0.6
        workspace_pose.pose.position.z = -0.19
        workspace_name = "workspace"
        self.scene.add_box(workspace_name, workspace_pose, size=(0.8, 0.5, 0.08))    

        self.scene.remove_attached_object(self.eef_link, name="parcel")
        self.scene.remove_world_object("parcel")

    def add_parcel(self, parcel):
        #add parcel to environment !
        parcel_pose = geometry_msgs.msg.PoseStamped()
        parcel_pose.header.frame_id = "base_link"
        parcel_pose.pose.position = Point(parcel.start_pos.x, parcel.start_pos.y, parcel.actual_size.z/100/2 - 0.014) #- 0.014 as the table is lower than the robot frame 
        parcel_pose.pose.orientation = self.euler_to_orientation(0, 0, parcel.angle)
        parcel_name = "parcel" + str(self.parcels_packed)

        self.scene.add_box(parcel_name, parcel_pose, size=(parcel.actual_size.x/100 -0.005, parcel.actual_size.y/100 -0.005, parcel.actual_size.z/100 -0.005)) #The parcel size is subtracted by 0.5 cm to parcels aren't packed direcly next to each other.

        self.update_end_goal(parcel.end_pos)
        self.go_to_pose("pick_ready")
        self.pick_parcel(parcel)



    def pick_parcel(self, parcel):
        print("boi")
        # x: rotation around the vertical axis | y: gripper to look down. | z: unused
        # x: -180 is the default rotation for the gripper
        # x: parcel.angle is the rotation of the parcel detected by the camera
        # x: parcel.parcel_rotation is for rotating the parcel +90 deg on the vertical axis
        #rot = Rotation.from_euler('xyz', [90 + parcel.angle + parcel.parcel_rotation, 90, 0], degrees=True)
        print("PARCEL ANGLE: ", parcel.angle)
        picking_pose = geometry_msgs.msg.Pose()
        picking_pose.position = Point(parcel.start_pos.x, parcel.start_pos.y, parcel.start_pos.z - 0.009) #- 0.009 as the table is lower than the robot frame 
        picking_pose.orientation = self.euler_to_orientation(-180, 0, 90 + parcel.angle)

        print("Pick Parcel Pose Goal: ", picking_pose.position)    
        self.group.set_pose_target(picking_pose)

        plan = self.group.go(wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        if plan==1:
            rospy.sleep(0.5)
            self.connect_parcel()
            rospy.sleep(0.5)
                        #self.go_to_pick_ready()          
            self.go_to_pose("pack_ready")
            self.place_parcel()
            self.detach_parcel()
            self.go_to_pose("pack_ready")
            self.go_to_pose("idle")

        self.group.clear_pose_targets()

        self.parcels_packed = self.parcels_packed + 1

    def update_end_goal(self, goal):

        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position = Point(goal.x, goal.y, goal.z) #- 0.009 as the table is lower than the robot frame 
        pose_goal.orientation = self.euler_to_orientation(-180, 0, 0)

        self.parcel_goal = pose_goal
       # print("Saving parcel place goal: ", self.parcel_goal)

    def place_parcel(self):
        print("Place parcel")

        print("Place parcel goal ", self.parcel_goal.position)    
        self.group.set_pose_target(self.parcel_goal)
    
        plan = self.group.go(wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        print("Did plan succeed: ", plan)

        return plan

    def clear_workspace(self):
        print("Clearing workspace")
        for i in range(10):
            self.scene.remove_world_object("parcel" + str(i))
            self.scene.remove_world_object("temp_parcel")

    def connect_parcel(self):
        print("connect to parcel!")
        grasping_group = 'manipulator'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, "parcel"+ str(self.parcels_packed), touch_links=touch_links)
        if self.sim == False:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io',SetIO)
            set_io(fun = 1, pin = 0 ,state = 1)


    def detach_parcel(self):
        print("detaching parcel!")
        self.scene.remove_attached_object(self.eef_link, name="parcel" + str(self.parcels_packed))
        self.scene.remove_world_object("parcel")
        if self.sim == False:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io',SetIO)
            set_io(fun = 1, pin = 0 ,state = 0)     

    def go_to_pose(self, pose):
        print("Going to ", pose)
        self.group.set_named_target(pose)
        plan = self.group.go(wait=True)
        self.group.stop()

        print("Did plan succeed: ", plan)

        return plan

    def euler_to_orientation(self, x, y, z): #Convert x y z degrees to a geometry_msg.Pose.orientation (a quaternion)
        rot = Rotation.from_euler('xyz', [x, y, z], degrees=True)
        rot_quat = rot.as_quat()

        output = Quaternion()

        output.x = rot_quat[0]
        output.y = rot_quat[1]
        output.z = rot_quat[2]
        output.w = rot_quat[3]

        return output


def main():
    #lav en instance af klassen
    mi = mover()

    rospy.spin()    


if __name__ == '__main__':
    main()
