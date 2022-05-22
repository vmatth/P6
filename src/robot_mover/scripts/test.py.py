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
        self.workspace = None

        rospy.sleep(2)

        self.add_environment()

        self.print_info()

        # self.go_to_predefined_pose("idle")

        # above_point = Point()
        # above_point.x = 0
        # above_point.y = -0.5
        # above_point.z = 0.27

        # self.go_to_point(above_point, self.euler_to_orientation(-180, 0, 90))

        # above_point = Point()
        # above_point.x = 0.25
        # above_point.y = -0.5
        # above_point.z = 0.27

        # self.go_to_point(above_point, self.euler_to_orientation(180, -90, 180))

        # above_point = Point()
        # above_point.x = 0.25
        # above_point.y = -0.5
        # above_point.z = 0.1

        # self.go_to_point(above_point, self.euler_to_orientation(180, -90, 180))



    def print_info(self):
        print("end effector: ", self.eef_link)
        
        print("planning frame: ", self.planning_frame)
        print("Robot Groups: ", self.robot.get_group_names())
        print("Robot State: ", self.robot.get_current_state())
        print("Robot Pose: ", self.group.get_current_pose())
        

    #Defines the environment in rviz for moveit trajectory calculations
    def add_environment(self):
        print("----------------------------------------")
        print("Adding environment")
        print("----------------------------------------")
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
        table_pose.pose.position.z = -0.059
        table_name = "table"
        self.scene.add_box(table_name, table_pose, size=(0.41, 0.93, 0.084))

        # stopper_pose = geometry_msgs.msg.PoseStamped()
        # stopper_pose.header.frame_id = "base_link"
        # stopper_pose.pose.orientation.w = 1.0
        # stopper_pose.pose.position.x = -0.18
        # stopper_pose.pose.position.y = -0.59
        # stopper_pose.pose.position.z = 0.03
        # stopper_name = "stopper"
        # self.scene.add_box(stopper_name, stopper_pose, size=(0.037, 0.38, 0.105))
        
        backboard_pose = geometry_msgs.msg.PoseStamped()
        backboard_pose.header.frame_id = "base_link"
        backboard_pose.pose.orientation.w = 1.0
        backboard_pose.pose.position.x = 0.0
        backboard_pose.pose.position.y = -0.78
        backboard_pose.pose.position.z = 0.445
        backboard_name = "backboard"
        self.scene.add_box(backboard_name, backboard_pose, size=(0.41, 0.05, 1.06))

        camholder_pose = geometry_msgs.msg.PoseStamped()
        camholder_pose.header.frame_id = "base_link"
        camholder_pose.pose.orientation.w = 1.0
        camholder_pose.pose.position.x = 0.0    
        camholder_pose.pose.position.y = -0.7
        camholder_pose.pose.position.z = 0.99
        calholder_name = "camera_holder"
        self.scene.add_box(calholder_name, camholder_pose, size=(0.03, 0.25, 0.03))

        camera_pose = geometry_msgs.msg.PoseStamped()
        camera_pose.header.frame_id = "base_link"
        camera_pose.pose.orientation.w = 1.0
        camera_pose.pose.position.x = 0.0    
        camera_pose.pose.position.y = -0.608
        camera_pose.pose.position.z = 0.9415
        camera_name = "camera"
        self.scene.add_box(camera_name, camera_pose, size=(0.249, 0.066, 0.067))    

    #This function moves the robot to "pose" which is a string containing the name of the pose defined in the moveit configuration
    def go_to_predefined_pose(self, pose):
        print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
        print("Going to ", pose)
        self.group.set_named_target(pose)
        plan = self.group.go(wait=True)
        self.group.stop()

        print("Did plan succeed: ", plan)
        print("&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&")
        return plan

    #Moves the robot to a pose.
    #Inputs are points (x,y,z) coordinates and orientation (x,y,z,w) quaternion.
    #Returns True or False depending on whether or not the movement succeded
    def go_to_point(self, point, orientation):
        print("Going to point ", point)
        p = geometry_msgs.msg.Pose()
        p.position = point
        p.orientation = orientation  
        self.group.set_pose_target(p)
        plan = self.group.go(wait=True)
        self.group.stop()    

        return plan 

    #Convert x y z degrees to a geometry_msg.Pose.orientation (a quaternion)
    def euler_to_orientation(self, x, y, z): 
        rot = Rotation.from_euler('xyz', [x, y, z], degrees=True)
        rot_quat = rot.as_quat()

        output = Quaternion()

        output.x = rot_quat[0]
        output.y = rot_quat[1]
        output.z = rot_quat[2]
        output.w = rot_quat[3]

        #print("quaternion pls work", output)

        return output

def main():
    #lav en instance af klassen
    mi = mover()

    rospy.spin()    


if __name__ == '__main__':
    main()
