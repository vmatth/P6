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
        rospy.Subscriber("/workspace/info", Workspace, self.workspace_callback)

        rospy.sleep(2)

        #self.print_info()

        self.add_environment()
        self.clear_workspace()
        
        #self.go_to_test_point(Point(-0.31, 0.5, 0.044))
        
        rospy.Subscriber("/robot/pick_place", Packing_info, self.add_parcel)

        print("Sim ", self.sim)
        self.go_to_predefined_pose("idle")

    def print_info(self):
        print("end effector: ", self.eef_link)
        
        print("planning frame: ", self.planning_frame)
        print("Robot Groups: ", self.robot.get_group_names())
        print("Robot State: ", self.robot.get_current_state())
        print("Robot Pose: ", self.group.get_current_pose())
        

    #This function is the callback function when receiving workspace info
    #This function adds the workspace to rviz using its size [cm] and center position [m]
    #Optional: The last line validates if the robot can move to the workspace corners.
    def workspace_callback(self, data):
        print("Receiving update from workspace callback")
        self.workspace = data
        workspace_pose = geometry_msgs.msg.PoseStamped()
        workspace_pose.header.frame_id = "base_link"
        workspace_pose.pose.orientation.w = 1.0
        workspace_pose.pose.position.x = data.center_position.x
        workspace_pose.pose.position.y = data.center_position.y
        workspace_pose.pose.position.z = data.center_position.z - 0.01 #Minus a small distance as this is defined in center points.
        workspace_name = "workspace"
        workspace_size = (data.size.y/100, data.size.x/100, 0.001)
        self.scene.add_box(workspace_name, workspace_pose, size=workspace_size)    
        #self.validate_workspace_boundaries() #todo only do once
        self.add_walls(workspace_pose, Point(workspace_size[0], workspace_size[1], data.size.z/100))


    #Defines the environment in rviz for moveit trajectory calculations
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

        self.scene.remove_attached_object(self.eef_link, name="parcel")
        self.scene.remove_world_object("parcel")

    def add_walls(self, workspace_pose, workspace_size):
        print("Adding walls")
        wall_thickness = 0.01
        wall_height = workspace_size.z
        wall_tolerance = 0.01


        wall_pose = geometry_msgs.msg.PoseStamped()
        wall_pose.header.frame_id = "base_link"
        wall_pose.pose.orientation.w = 1.0
        wall_pose.pose.position.x = workspace_pose.pose.position.x
        wall_pose.pose.position.y = workspace_pose.pose.position.y + (workspace_size.y/2) + (wall_thickness/2) + wall_tolerance
        wall_pose.pose.position.z = workspace_pose.pose.position.z + (wall_height/2)
        wall_name = "wall_0"
        self.scene.add_box(wall_name, wall_pose, size=(workspace_size.x, wall_thickness, wall_height))    

        wall_pose = geometry_msgs.msg.PoseStamped()
        wall_pose.header.frame_id = "base_link"
        wall_pose.pose.orientation.w = 1.0
        wall_pose.pose.position.x = workspace_pose.pose.position.x + (workspace_size.x/2) + (wall_thickness/2) + wall_tolerance
        wall_pose.pose.position.y = workspace_pose.pose.position.y + wall_tolerance
        wall_pose.pose.position.z = workspace_pose.pose.position.z + (wall_height/2)
        wall_name = "wall_1"
        self.scene.add_box(wall_name, wall_pose, size=(wall_thickness, workspace_size.y, wall_height))      

        wall_pose = geometry_msgs.msg.PoseStamped()
        wall_pose.header.frame_id = "base_link"
        wall_pose.pose.orientation.w = 1.0
        wall_pose.pose.position.x = workspace_pose.pose.position.x - (workspace_size.x/2) - (wall_thickness/2) - wall_tolerance
        wall_pose.pose.position.y = workspace_pose.pose.position.y + wall_tolerance
        wall_pose.pose.position.z = workspace_pose.pose.position.z + (wall_height/2)
        wall_name = "wall_2"
        self.scene.add_box(wall_name, wall_pose, size=(wall_thickness, workspace_size.y, wall_height))     

    #This function is the callback function when receiving a new packing info from the packing algorithm
    #Adds the parcel to rviz and calls another function that begins picking the added parcel
    def add_parcel(self, parcel):
        print("adding parcel at pos", Point(parcel.start_pos.x, parcel.start_pos.y, parcel.non_rotated_size.z/100/2 - 0.014))
        #add parcel to environment !
        parcel_pose = geometry_msgs.msg.PoseStamped()
        parcel_pose.header.frame_id = "base_link"
        parcel_pose.pose.position = Point(parcel.start_pos.x, parcel.start_pos.y, parcel.non_rotated_size.z/100/2 - 0.014) #- 0.014 as the table is lower than the robot frame 
        parcel_pose.pose.orientation = self.euler_to_orientation(0, 0, 90 + abs(parcel.angle))
        parcel_name = "parcel" + str(self.parcels_packed)

        self.scene.add_box(parcel_name, parcel_pose, size=(parcel.non_rotated_size.x/100 -0.005, parcel.non_rotated_size.y/100 -0.005, parcel.non_rotated_size.z/100 -0.005)) #The parcel size is subtracted by 0.5 cm to parcels aren't packed direcly next to each other.

        self.update_end_goal(parcel.end_pos, parcel.parcel_rotation)
        self.go_to_predefined_pose("pick_ready")
        self.pick_parcel(parcel)



    #This function handles all of the pick and place logic for the robotic arm
    #Starts by calculating the picking position and orientation of the parcel
    #Thereafter, picks, connects, places, detacthes and returns to idle
    def pick_parcel(self, parcel):
        #Pick parcel from above
        plan = 0
        if parcel.picking_side == 1:
            plan = self.go_to_point(Point(parcel.start_pos.x, parcel.start_pos.y, parcel.start_pos.z - 0.009), self.euler_to_orientation(-180, 0, 90 + abs(parcel.angle)))
            #rospy.sleep(0.25)
            self.connect_parcel()
            self.suck()
        #Pick parcel from side
        elif parcel.picking_side == 2 and parcel.non_rotated_size.y > parcel.non_rotated_size.x:
            print("pick side2, y>x")
            #Rotate parcel to 0 degrees
            if self.rotate_parcel_to_angle(parcel, 90):
                plan = self.grab_parcel_at_picking_side(parcel, parcel.non_rotated_size.x/100, 0, False)
        elif parcel.picking_side == 2 and parcel.non_rotated_size.x > parcel.non_rotated_size.y:
            print("pick side 2, x>y")
            #Rotate parcel to 90 degrees
            if self.rotate_parcel_to_angle(parcel, 0):
                plan = self.grab_parcel_at_picking_side(parcel, parcel.non_rotated_size.y/100, 90, False)            
        elif parcel.picking_side == 3 and parcel.non_rotated_size.y > parcel.non_rotated_size.x:
            print("pick side 3, y>x")
            #Rotate parcel to 90 degrees
            if self.rotate_parcel_to_angle(parcel, 90):
                plan = self.grab_parcel_at_picking_side(parcel, parcel.non_rotated_size.x/100, 0, True)
        elif parcel.picking_side == 3 and parcel.non_rotated_size.x > parcel.non_rotated_size.y:
            print("pick side 3, x>y")
            #Rotate parcel to 0 degrees
            if self.rotate_parcel_to_angle(parcel, 0):
                plan = self.grab_parcel_at_picking_side(parcel, parcel.non_rotated_size.y/100, 90, False)

        #If the plan succeeded then pack
        if plan == 1:
            #rospy.sleep(0.25)         
            self.go_to_predefined_pose("pack_ready")
            self.go_to_above_pack()
            self.place_parcel()
            self.detach_parcel()
            self.go_to_predefined_pose("pack_ready")
            self.go_to_predefined_pose("idle")
            self.group.clear_pose_targets()
            self.parcels_packed = self.parcels_packed + 1
        else:
            print("Plan Failed")


    #This function rotates the parcel to a specificed angle (degrees) which simplifies picking different sides
    #Returns True or False (if we succeeded)
    def rotate_parcel_to_angle(self, parcel, ang):
        print("%%%%%%%%%%%%%%%%%%%%%%%%%%")
        print("Current parcel angle is: ", abs(parcel.angle))
        print("Desired angle is, ", ang)
        ang_diff = ang - abs(parcel.angle)
        print("Angle difference ", ang_diff)
        if(abs(ang_diff) < 1):
                return True
        #Pick the parcel
        if self.go_to_point(Point(parcel.start_pos.x, parcel.start_pos.y, parcel.start_pos.z - 0.009), self.euler_to_orientation(-180, 0, 90 + abs(parcel.angle))):  
            #rospy.sleep(0.5)
            self.connect_parcel()
            self.suck()
            #rospy.sleep(0.5)
            #Rotate the parcel
            if self.go_to_point(Point(parcel.start_pos.x, parcel.start_pos.y, parcel.start_pos.z - 0.009), self.euler_to_orientation(-180, 0, 90 + ang)):  
                self.detach_parcel()
                #rospy.sleep(0.5)

                above_point = Point()
                above_point = self.group.get_current_pose().pose.position
                above_point.z = above_point.z + 0.15

                self.go_to_point(above_point, self.group.get_current_pose().pose.orientation)
                #self.go_to_predefined_pose("pick_ready") #TODO: DONT GO ALL THE WAY UP!
                return True
        return False

    #Grabs that parcel from the left side
    #Returns if succeeded
    #Angle is the parcel angle after rotation  in rviz
    #rot angle is 0 or 90 depending on if the gripper needs to pick up the parcel rotated
    def grab_parcel_at_picking_side(self, parcel, new_width, angle, rot_angle):
        grab_offset = 0.01 #The gripper pushes the parcel slightly to make sure it has grabbed it [m]
        #Go to position next to the parcel
        self.go_to_point(Point(parcel.start_pos.x + new_width, parcel.start_pos.y, parcel.start_pos.z - (parcel.non_rotated_size.z/100/2)), self.euler_to_orientation(180, -90, 180))  
        if rot_angle == True:
            self.go_to_point(Point(parcel.start_pos.x + new_width, parcel.start_pos.y, parcel.start_pos.z - (parcel.non_rotated_size.z/100/2)), Quaternion(-0.5, -0.5, 0.5, 0.5))  
        #Remove parcel from collision (temporary)
        parcel_name = "parcel" + str(self.parcels_packed)
        self.scene.remove_world_object(parcel_name)
        #Grab parcel
        self.suck()
        # #Go to parcel
        if rot_angle == False:
            self.go_to_point(Point(parcel.start_pos.x + (new_width/2) - grab_offset, parcel.start_pos.y, parcel.start_pos.z - (parcel.non_rotated_size.z/100/2)), self.euler_to_orientation(180, -90, 180))
        else:
            self.go_to_point(Point(parcel.start_pos.x + (new_width/2) - grab_offset, parcel.start_pos.y, parcel.start_pos.z - (parcel.non_rotated_size.z/100/2)), Quaternion(-0.5, -0.5, 0.5, 0.5))
        # #Re-Add parcel to collision
        gripper_pose = self.group.get_current_pose() #First we find where to gripper is located
        parcel_pose = geometry_msgs.msg.PoseStamped()
        parcel_pose.header.frame_id = "base_link"
        parcel_pose.pose.position = Point(gripper_pose.pose.position.x - (new_width/2), parcel.start_pos.y, parcel.non_rotated_size.z/100/2 - 0.014) #- 0.014 as the table is lower than the robot frame 
        parcel_pose.pose.orientation = self.euler_to_orientation(0, 0, angle)
        self.scene.add_box(parcel_name, parcel_pose, size=(parcel.non_rotated_size.x/100 -0.005, parcel.non_rotated_size.y/100 -0.005, parcel.non_rotated_size.z/100 -0.005)) #The parcel size is subtracted by 0.5 cm to parcels aren't packed direcly next to each other.
        #Connect parcel to move it collision
        self.connect_parcel()
        #rospy.sleep(0.25)
        #Move slightly up to prevent collision with table
        #plan = self.go_to_point(Point(parcel.start_pos.x + (parcel.non_rotated_size.y/100/2), parcel.start_pos.y, parcel.start_pos.z + 0.2), self.euler_to_orientation(180, -90, 180))
        plan = True
        return plan

    #Stores the end_goal in the local variable: "self.parcel_goal"
    #This goal is used for later when the robot needs to place a parcel
    def update_end_goal(self, goal, rotation):

        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position = Point(goal.x, goal.y, goal.z) #- 0.009 as the table is lower than the robot frame 
        pose_goal.orientation = self.euler_to_orientation(-180, 0, -90 + rotation)

        self.parcel_goal = pose_goal
       # print("Saving parcel place goal: ", self.parcel_goal)

    #Goes to the stored self.parcel_goal position
    def place_parcel(self):
        print("Place parcel")

        print("Place parcel goal ", self.parcel_goal.position)    
        self.group.set_pose_target(self.parcel_goal)
    
        plan = self.group.go(wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        self.group.stop()

        print("Did plan succeed: ", plan)

        return plan

    #Clears all packed parcels
    def clear_workspace(self):
        print("Clearing workspace")
        for i in range(10):
            try:
                self.scene.remove_attached_object(self.eef_link, name="parcel" + str(i))
            except:
                print(" ")
            try:
                self.scene.remove_world_object("parcel" + str(i))
            except:
                print(" ")

    #Connects the parcel to the moveit collision system in rviz
    def connect_parcel(self):
        print("connect to parcel!")
        grasping_group = 'manipulator'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, "parcel"+ str(self.parcels_packed), touch_links=touch_links)

    #Starts the vacuum gripper (only if not simulating)
    def suck(self):
        if self.sim == False:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io',SetIO)
            set_io(fun = 1, pin = 0 ,state = 1)


    #Detaches the parcel from the moveit collision system in rviz and the physical vacuum gripper
    def detach_parcel(self):
        print("detaching parcel!")
        self.scene.remove_attached_object(self.eef_link, name="parcel" + str(self.parcels_packed))
        if self.sim == False:
            set_io = rospy.ServiceProxy('/ur_hardware_interface/set_io',SetIO)
            set_io(fun = 1, pin = 0 ,state = 0)     

    #This function moves the robot to "pose" which is a string containing the name of the pose defined in the moveit configuration
    def go_to_predefined_pose(self, pose):
        print("Going to ", pose)
        self.group.set_named_target(pose)
        plan = self.group.go(wait=True)
        self.group.stop()

        print("Did plan succeed: ", plan)

        return plan

    #This function moves the robot above the position where the parcel will be packed
    #This is to help with the trajectory planning as the robot will only need to move downwards along the z-axis
    def go_to_above_pack(self):
        print("Moving above pack")
        #above_pose = geometry_msgs.msg.PoseStamped()
        above_pose = geometry_msgs.msg.Pose()

        above_pose.position.x = self.parcel_goal.position.x
        above_pose.position.y = self.parcel_goal.position.y
        above_pose.position.z = self.parcel_goal.position.z + 0.1

        above_pose.orientation = self.parcel_goal.orientation
        
        print("Above Pose: ", above_pose.position)

        self.group.set_pose_target(above_pose)
        plan = self.group.go(wait=True)
        self.group.stop()  


    #This function moves the robot to each corner in the workspace
    #The purpose is to check if the robot has enough range for the given workspace
    def validate_workspace_boundaries(self):
        print("Validating workspace boundaries")
        p = geometry_msgs.msg.Pose()
        p.position = Point(self.workspace.corner_position.x, self.workspace.corner_position.y, 0) #- 0.009 as the table is lower than the robot frame 
        p.orientation = self.euler_to_orientation(-180, 0, 0)
        print("First val position: ", p.position)    
        self.group.set_pose_target(p)
        plan = self.group.go(wait=True)
        self.group.stop()

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

        print("quaternion pls work", output)

        return output


def main():
    #lav en instance af klassen
    mi = mover()

    rospy.spin()    


if __name__ == '__main__':
    main()
