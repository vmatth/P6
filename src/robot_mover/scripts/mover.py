#! /usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Pose  
from read_camera.msg import Parcel #Parcel msg
from scipy.spatial.transform import Rotation


#class here
class mover:

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_mover', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "manipulator"
        self.group = moveit_commander.MoveGroupCommander(group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

        planning_frame = self.group.get_planning_frame()
        print("planning frame: ", planning_frame)

        self.eef_link = self.group.get_end_effector_link()
        print("end effector: ", self.eef_link)

        group_names = self.robot.get_group_names()
        print("Robot Groups: ", self.robot.get_group_names())

        print("Robot State: ", self.robot.get_current_state())

        print("Robot Pose: ", self.group.get_current_pose())
    
        #rospy.Subscriber("/vision/parcel_raw", Parcel, self.movement_callback)
        #rospy.Subscriber("/robot/pose", Pose, self.movement_callback)
        #subscribe to parcel here
        rospy.Subscriber("/vision/parcel_raw", Parcel, self.add_parcel)


        self.add_environment()
        #transform_camera = [1, 0, 0, -0.1
        #                     0, -1, 0, 0.54
        #                     0, 0,-1, 1.02]
        

    def add_parcel(self, parcel):
        rospy.loginfo(rospy.get_caller_id() + "New parcel %s", parcel)

        #rad to quaternion
        rot = Rotation.from_euler('xyz', [180, 0, parcel.angle], degrees=True)
        rot_quat = rot.as_quat()
        print(rot_quat)

         #add to environment !
        parcel_pose = geometry_msgs.msg.PoseStamped()
        parcel_pose.header.frame_id = "base"
        parcel_pose.pose.orientation.x = rot_quat[0]
        parcel_pose.pose.orientation.y = rot_quat[1]
        parcel_pose.pose.orientation.z = rot_quat[2] #parcel.angle
        parcel_pose.pose.orientation.w = rot_quat[3] #quaternion
        parcel_pose.pose.position.x = parcel.centerpoint.x
        parcel_pose.pose.position.y = parcel.centerpoint.y 
        parcel_pose.pose.position.z = parcel.size.z/2 -0.03
        parcel_name = "parcel"
        self.scene.add_box(parcel_name, parcel_pose, size=(parcel.size.x, parcel.size.y, parcel.size.z))

        #*1 here and stuf
        parcel_pose.pose.position.x = parcel.centerpoint.x * -1
        parcel_pose.pose.position.y = parcel.centerpoint.y * -1

        self.pick_parcel(parcel_pose.pose, parcel.size)


    def add_environment(self):
        table_pose = geometry_msgs.msg.PoseStamped()
        table_pose.header.frame_id = "base"
        table_pose.pose.orientation.w = 1.0
        table_pose.pose.position.x = 0
        table_pose.pose.position.y = 0.3
        table_pose.pose.position.z = -0.03
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=(0.42, 0.93, 0.05))
        
        backboard_pose = geometry_msgs.msg.PoseStamped()
        backboard_pose.header.frame_id = "base"
        backboard_pose.pose.orientation.w = 1.0
        backboard_pose.pose.position.x = 0.0
        backboard_pose.pose.position.y = 0.74   
        backboard_pose.pose.position.z = 0.47
        backboard_name = "backboard"
        self.scene.add_box(backboard_name, backboard_pose, size=(0.42, 0.05, 1.0))


    def connect_parcel(self):
        print("connect to parcel!")
        grasping_group = 'endeffector'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, "parcel", touch_links=touch_links)
        print("Parcel Connected?")


    def pick_parcel(self, pose, size):
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAH MOVEMENT")
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose)
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal = pose #D:
        pose_goal.position.z =+ size.z #maaske / 2
        print("pls go to: ", pose_goal)    
        self.group.set_pose_target(pose_goal)
    
        plan = self.group.go(wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()


        print("PLAN!!", plan)

        if plan == 1 :
            self.connect_parcel()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        #self.group.clear_pose_targets()
        print("arrived!")



        # plan, frac = self.damn()
        # self.group.execute(plan, wait=True)
        print("arrived")
        #self.scene.remove_world_object("parcel")


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
