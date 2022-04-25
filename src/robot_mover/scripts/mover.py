#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose  
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

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self.parcel_goal = geometry_msgs.msg.Pose()

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        planning_frame = self.group.get_planning_frame()
        print("planning frame: ", planning_frame)

        self.eef_link = self.group.get_end_effector_link()
        print("end effector: ", self.eef_link)

        group_names = self.robot.get_group_names()
        print("Robot Groups: ", self.robot.get_group_names())

        print("Robot State: ", self.robot.get_current_state())

        print("Robot Pose: ", self.group.get_current_pose())

        self.add_environment()
        
        #rospy.Subscriber("/vision/parcel_raw", Parcel, self.movement_callback)
        #rospy.Subscriber("/robot/pose", Pose, self.movement_callback)
        #subscribe to parcel here
        #rospy.Subscriber("/vision/parcel_raw", Parcel, self.add_parcel)
        rospy.Subscriber("/robot/pick_place", Packing_info, self.add_parcel)
        rospy.Subscriber("/workspace/info", Workspace, self.add_workspace)

        #print("Named Targets: , ", self.group.get_named_targets())

        self.go_to_pick_ready()
        
    def add_environment(self):
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "base"
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = 0
        table_pose.pose.position.y = 0.31
        table_pose.pose.position.z = -0.045
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=(0.41, 0.93, 0.08))
        
        backboard_pose = geometry_msgs.msg.PoseStamped()
        backboard_pose.header.frame_id = "base"
        backboard_pose.pose.orientation.w = 1.0
        backboard_pose.pose.position.x = 0.0
        backboard_pose.pose.position.y = 0.80   
        backboard_pose.pose.position.z = 0.445
        backboard_name = "backboard"
        self.scene.add_box(backboard_name, backboard_pose, size=(0.41, 0.05, 1.06))

        #super unnecessary but maybe kinda work idk
        # ceiling_pose = geometry_msgs.msg.PoseStamped()
        # ceiling_pose.header.frame_id = "base"
        # ceiling_pose.pose.orientation.w = 1.0
        # ceiling_pose.pose.position.x = 0.0
        # ceiling_pose.pose.position.y = 0.31
        # ceiling_pose.pose.position.z = 1.0
        # ceiling_name = "ceiling"
        # self.scene.add_box(ceiling_name, ceiling_pose, size=(3.41, 3.0, 0.08))

        #workspace size = 60x80
        #workspace center x from robot frame: 60
        #workspace z from robot frame: -20

        self.scene.remove_attached_object(self.eef_link, name="parcel")
        self.scene.remove_world_object("parcel")

    def add_workspace(self, data):
        workspace_pose = geometry_msgs.msg.PoseStamped()
        workspace_pose.header.frame_id = "base"
        workspace_pose.pose.orientation.w = 1.0
        workspace_pose.pose.position.x = 0
        workspace_pose.pose.position.y = 0.31
        workspace_pose.pose.position.z = -0.045
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=(0.41, 0.93, 0.08))        



    def add_parcel(self, parcel):
        #rad to quaternion
        rot = Rotation.from_euler('xyz', [180, 0, parcel.angle], degrees=True)
        rot_quat = rot.as_quat()
        #print(rot_quat)

        #add parcel to environment !
        parcel_pose = geometry_msgs.msg.PoseStamped()
        parcel_pose.header.frame_id = "base"

        parcel_pose.pose.position.x = parcel.start_pos.x
        parcel_pose.pose.position.y = parcel.start_pos.y 
        parcel_pose.pose.position.z = parcel.start_pos.z - parcel.size.z
        parcel_pose.pose.orientation.x = rot_quat[0]
        parcel_pose.pose.orientation.y = rot_quat[1]
        parcel_pose.pose.orientation.z = rot_quat[2] #parcel.angle
        parcel_pose.pose.orientation.w = rot_quat[3] #quaternion
        parcel_name = "parcel"
        self.scene.add_box(parcel_name, parcel_pose, size=(parcel.size.x, parcel.size.y, parcel.size.z))

        #pick pose here kan du se at jeg skriver her viniboi no
        picking_pose = geometry_msgs.msg.PoseStamped()

        # if parcel.picking_side == 1:
        #     print("Picking side ", 1)
        #     #rotate the gripper based on which rotation the packing algorithm gives
        #     #rot = Rotation.from_euler('xyz', [180, 0, parcel.angle + parcel.parcel_rotation], degrees=True)
        #     rot = Rotation.from_euler('xyz', [parcel.angle + parcel.parcel_rotation, 90, 0], degrees=True)
        #     rot_quat = rot.as_quat()
        #     picking_pose.pose.position.x = parcel.start_pos.x *-1
        #     picking_pose.pose.position.y = parcel.start_pos.y *-1
        #     picking_pose.pose.position.z = parcel.size.z/2 #-0.03
        #     picking_pose.pose.orientation.x = rot_quat[0]
        #     picking_pose.pose.orientation.y = rot_quat[1]
        #     picking_pose.pose.orientation.z = rot_quat[2] #parcel.angle
        #     picking_pose.pose.orientation.w = rot_quat[3] #quaternion
        # #find ud af om det er picking side 2
        # if parcel.picking_side == 2:
        #     print("Picking side ", 2)
        #     rot = Rotation.from_euler('xyz', [90, 90, 0], degrees=True)
        #     rot_quat = rot.as_quat()

        #     print("start pos x y: ", parcel.start_pos.x *-1, parcel.start_pos.y *-1)

        #     print("cos stuff", (math.cos(math.radians(parcel.parcel_rotation))))
        #     print("size", parcel.size.x)

        #     picking_pose.pose.position.x = parcel.start_pos.x*-1 - (math.cos(math.radians(parcel.parcel_rotation))* parcel.size.x/2)
        #     picking_pose.pose.position.y = parcel.start_pos.y*-1 - (math.sin(math.radians(parcel.parcel_rotation))* parcel.size.y/2)

        #     print("new pos with cos sin x y ", picking_pose.pose.position.x, picking_pose.pose.position.y)

        #     picking_pose.pose.position.z = parcel.size.z/2 #-0.03 todoo fix /2
        #     picking_pose.pose.orientation.x = rot_quat[0]
        #     picking_pose.pose.orientation.y = rot_quat[1]
        #     picking_pose.pose.orientation.z = rot_quat[2] #parcel.angle
        #     picking_pose.pose.orientation.w = rot_quat[3] #quaternion

        self.update_end_goal(parcel.end_pos)
        self.pick_parcel(picking_pose.pose, parcel.size)

    def connect_parcel(self):
        print("connect to parcel!")
        grasping_group = 'endeffector'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, "parcel", touch_links=touch_links)
        set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io',SetIO)
        set_io(fun = 1, pin = 0 ,state = 1)
        print("Parcel Connected?")


    def go_to_pick_ready(self):
        print("Going to pick_ready")
        self.group.set_named_target("pick_ready")
    
        plan = self.group.go(wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        self.group.clear_pose_targets()

        print("Did plan succeed: ", plan)

        return plan

    def go_to_pack_ready(self):
        print("Going to pack_ready")
        self.group.set_named_target("pack_ready")
    
        plan = self.group.go(wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        self.group.clear_pose_targets()

        print("Did plan succeed: ", plan)

        return plan

    def pick_parcel(self, pose, size):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal = pose
        pose_goal.position.z =+ size.z + 0.001
        print("Pick Parcel Pose Goal: ", pose_goal)    
        self.group.set_pose_target(pose_goal)

        plan = self.group.go(wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        self.group.clear_pose_targets()

        if plan==1:

            rospy.sleep(0.5)
            self.connect_parcel()
            self.go_to_pick_ready()          
            self.go_to_pack_ready()
            self.place_parcel()
        # if plan == 1:
        #     self.pick_parcel(self.parcel_goal)
        #     rospy.sleep(2)

    def update_end_goal(self, goal):
        #rad to quaternion
        rot = Rotation.from_euler('xyz', [0, 90, 0], degrees=True)
        rot_quat = rot.as_quat()

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = self.group.get_current_pose().pose.orientation

        # pose_goal.orientation.x = rot_quat[0]
        # pose_goal.orientation.y = rot_quat[1]
        # pose_goal.orientation.z = rot_quat[2] #parcel.angle
        # pose_goal.orientation.w = rot_quat[3] #quaternion

        pose_goal.position.x = goal.x
        pose_goal.position.y = goal.y
        pose_goal.position.z = goal.z

        self.parcel_goal = pose_goal
        print("Saving parcel place goal: ", self.parcel_goal)

    def place_parcel(self):
        print("Place parcel")

        print("Place parcel goal ", self.parcel_goal)    
        self.group.set_pose_target(self.parcel_goal)
    
        plan = self.group.go(wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        print("Did plan succeed: ", plan)
        if plan == 1:
            self.scene.remove_attached_object(self.eef_link, name="parcel")
            self.scene.remove_world_object("parcel")
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io',SetIO)
            set_io(fun = 1, pin = 0 ,state = 0)          
            self.go_to_pack_ready()
            self.go_to_pick_ready()

        self.group.clear_pose_targets()


def main():
    #lav en instance af klassen
    mi = mover()

    # We can get the joint values from the group and adjust some of the values:
    # joint_goal = group.get_current_joint_values()
    # joint_goal[0] = 0
    # joint_goal[1] = -pi/4
    # joint_goal[2] = 0
    # joint_goal[3] = -pi/2
    # joint_goal[4] = 0
    # joint_goal[5] = pi/3
    #joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    #group.go(joint_goal, wait=True)


    rospy.spin()    


if __name__ == '__main__':
    main()
