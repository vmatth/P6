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

        eef_link = self.group.get_end_effector_link()
        print("end effector: ", eef_link)

        group_names = self.robot.get_group_names()
        print("Robot Groups: ", self.robot.get_group_names())

        print("Robot State: ", self.robot.get_current_state())

        print("Robot Pose: ", self.group.get_current_pose())
    
        #rospy.Subscriber("/vision/parcel_raw", Parcel, self.movement_callback)
        rospy.Subscriber("/robot/pose", Pose, self.movement_callback)
        #subscribe to parcel here
        rospy.Subscriber("/vision/parcel_raw", Parcel, self.add_parcel)

        self.add_environment()
        #transform_camera = [1, 0, 0, -0.1
        #                     0, -1, 0, 0.54
        #                     0, 0,-1, 1.02]

    def add_parcel(self, parcel):
        rospy.loginfo(rospy.get_caller_id() + "New parcel %s", parcel)

        #rad to quaternion
        rot = Rotation.from_euler('xyz', [0, 0, parcel.angle], degrees=True)
        rot_quat = rot.as_quat()
        print(rot_quat)

         #add to environment !
        parcel_pose = geometry_msgs.msg.PoseStamped()
        parcel_pose.header.frame_id = "base"
        parcel_pose.pose.orientation.w = 1 #quaternion
        parcel_pose.pose.orientation.z = rot_quat #parcel.angle
        parcel_pose.pose.position.x = parcel.centerpoint.x
        parcel_pose.pose.position.y = parcel.centerpoint.y
        parcel_pose.pose.position.z = parcel.size.z/2 -0.03
        parcel_name = "parcel"
        self.scene.add_box(parcel_name, parcel_pose, size=(parcel.size.x, parcel.size.y, parcel.size.z))


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

    def movement_callback(self, pose):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", pose)
        # pose_goal = geometry_msgs.msg.Pose()
        # # pose_goal.orientation.x = -0.033#0.7071
        # # pose_goal.orientation.y = 0.70631#0.7071
        # # pose_goal.orientation.z = -0.70631 #Parcel.angle
        # pose_goal.orientation.w = 1.0
        # #pose_goal.positiosffsfhhjhjhjn.x = Parcel.centerpoint.x
        # #pose_goal.position.y = Parcel.centerpoint.y
        # #pose_goal.position.z = Parcel.centerpoint.z
        # #print("wtf",Parcel.centerpoint.x/100*-1)
        # #Calculate goal pos using the transformation frame. Converts from cm to m
        # # pose_goal.position.x = (Parcel.centerpoint.x / 100 *  1) + 0.1
        # # pose_goal.position.y = (Parcel.centerpoint.y / 100 * -1) + 0.54
        # # pose_goal.position.z = (Parcel.centerpoint.z / 100 * -1) + 1.02 #0.0 -0.20 1<<
        # pose_goal.position.x = Parcel.centerpoint.x
        # pose_goal.position.y = Parcel.centerpoint.y
        # # pose_goal.position.z = Parcel.centerpoint.z
        print("pls go to: ", pose)
        self.group.set_pose_target(pose)
        
        plan = self.group.go(wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()


        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.group.clear_pose_targets()
        print("arrived!")



        # plan, frac = self.damn()
        # self.group.execute(plan, wait=True)
        # print("arrived")



    def damn(self):
        print("cartesian paths")
        waypoints = []
        scale = 1.0

        wpose = self.group.get_current_pose().pose
        wpose.position.z += scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # wpose = self.group.get_current_pose().pose
        # wpose.position.x = 0.2
        # wpose.position.y = -0.2
        # wpose.position.y = 0.5
        # waypoints.append(copy.deepcopy(wpose))

        # wpose = self.group.get_current_pose().pose
        # wpose.position.x = 0.4
        # wpose.position.y = 0.0
        # wpose.position.y = 0.5
        # waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

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
