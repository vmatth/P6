#!/usr/bin/env python
import rospy
from bin_packing.msg import Workspace #workspace msg
from bin_packing.parcel import parcel #parcel class
from geometry_msgs.msg import Point #point type
from bin_packing.msg import Packing_info #Packing msg
from bin_packing.msg import Height_Map_Row
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from bin_packing.convertTo2DArray import convertTo2DArray #convert function
from bin_packing.convertTo2DArray import convertToMultiArray #convert function
import csv

class workspace:
    def __init__ (self, x, y, z, center_pos, corner_pos):
        self.parcels = []
        self.workspace_size = Point(x,y,z)
        self.center_position = center_pos
        self.corner_position = corner_pos
        self.height_map = [[0.0 for i in range(y)] for j in range(x)] 
        self.sub = rospy.Subscriber("/workspace/add_parcel", Packing_info, self.add_parcel)
        self.pub = rospy.Publisher("/workspace/info", Workspace, queue_size=10, latch=True)
        self.sub = rospy.Subscriber("/workspace/update_height_map", Workspace, self.heightmap_callback)
        self.sub = rospy.Subscriber("/workspace/remove_parcels", Workspace, self.remove_parcels)
        self.counter = 0
        self.seed = 1

        self.update_workspace()

        print("Creating new workspace with size. ", self.workspace_size)

    def add_parcel(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Workspace received new parcel %s", data)
        #create a parcel class
        self.parcels.append(data)

        #Add parcel to height map, by changing each pixel (x,y) to the height
        for x in range(int(data.end_pos.x), int(data.end_pos.x) + int(data.rounded_size.x)):
            for y in range(int(data.end_pos.y), int(data.end_pos.y) + int(data.rounded_size.y)):
                self.height_map[x][y] = int(data.rounded_size.z) + int(data.end_pos.z)
            
        self.update_workspace()
        
    #Workspace is updated each time a new parcel is added.
    def update_workspace(self):
        #Publish to workspace/info
        msg = Workspace()
        msg.size = self.workspace_size
        msg.center_position = self.center_position
        msg.corner_position = self.corner_position

        hm = convertToMultiArray(self.height_map, self.workspace_size.x)

        msg.height_map = hm
        msg.parcels = self.parcels
        self.pub.publish(msg)

        #Calculate Volume
        parcelsV = []
        height_parcel_list = []
        length_parcel_list = []
        width_parcel_list = []
        for i in range(len(self.parcels)):
            parcel_volume = self.parcels[i].rounded_size.x * self.parcels[i].rounded_size.y * self.parcels[i].rounded_size.z 
            parcelsV.append(parcel_volume)
            height_of_parcels = self.parcels[i].end_pos.z + self.parcels[i].rounded_size.z
            height_parcel_list.append(height_of_parcels)
            lenght_of_parcels = self.parcels[i].end_pos.x + self.parcels[i].rounded_size.x
            length_parcel_list.append(lenght_of_parcels)
            width_of_parcels = self.parcels[i].end_pos.y + self.parcels[i].rounded_size.y
            width_parcel_list.append(width_of_parcels)

        c_workspace_height = 0
        c_workspace_length = 0
        c_workspace_width = 0
        if len(height_parcel_list) > 0:    
            c_workspace_height = max(height_parcel_list)
            c_workspace_length = max(length_parcel_list)
            c_workspace_width = max(width_parcel_list)
            #print("c_workspace_height: ", c_workspace_height)
        print("-----------------------------------------------------")
        #print("Parcel_volume: ", parcel_volume)
        #print("list of volumes: ", parcelsV)
        parcels_combined_volume = sum(parcelsV)
        #print("Combined Volume: ", parcels_combined_volume)
        v_workspace = self.workspace_size.x * self.workspace_size.y * self.workspace_size.z
        print("Workspace Volume: ", v_workspace)
        fill_rate = 0
        self.fill_rate = 0
        num_parcels = 0
        self.num_parcels = 0
        self.compactness = 0
        if parcels_combined_volume > 0:
            fill_rate = int(parcels_combined_volume / v_workspace  * 100)
            print("fill_rate: ", fill_rate, "%")
            num_parcels = len(parcelsV)
            self.num_parcels = num_parcels
            self.fill_rate = fill_rate

        #Calculate Compactness
        c_workspace = c_workspace_length * c_workspace_width * c_workspace_height
        print("C_workspace: ", c_workspace)
        compactness = 0
        if parcels_combined_volume > 0:
            compactness = int(parcels_combined_volume / c_workspace * 100)
            print("compactness: ", compactness, "%")
            print("Parcel nr.: ", num_parcels)
            self.compactness = compactness
        print("-----------------------------------------------------")
        # pakkens z position plus pakkens height 

        #save to self



    def heightmap_callback(self, data):
        print("Height map updated with size: ", data.size)
        print("receiving hm", data.height_map)
        self.workspace_size = data.size
        self.height_map = convertTo2DArray(data.height_map, False)
        self.update_workspace()


    def remove_parcels(self, data):
        #Save data here
        if data.save_data == True:
            print("Saving workspace data")

            if self.counter == 0:
                header = ['Fill_rate', 'Compactness', 'Items', 'Seed']
                #data = [str(fill_rate), str(compactness), str(items)]

                with open('packing_test.csv', 'w') as f:
                    writer = csv.writer(f)
                    writer.writerow(header)
                    self.counter = 1

            if self.counter == 1:
                dota = [str(self.fill_rate), str(self.compactness), str(self.num_parcels), str(self.seed)]
                #dota = ['vini', 'is', 'hot']
                with open('packing_test.csv', 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow(dota)


            

        #Remove parcels from workspace
        if data.parcels_to_yeet == -1:
            print("Clearing workspace")
            self.parcels = []
            self.height_map = [[0.0 for i in range(self.workspace_size.y)] for j in range(self.workspace_size.x)] 
            rospy.sleep(2)
            self.update_workspace()

        self.seed = self.seed + 1
        ##########################
        #########TESTING
        ##########################
        if(self.seed == 200 + 1): ###Change this number for the amount of times to test
            print("STOPPING TEST!!!")
            rospy.sleep(999999)

def main():
    rospy.init_node('workspace', anonymous=True)
    #rosrun pkg node _x:=2 _y:5 _z:=10
    #size in cm

    # #Size at UR5 setup
    x = rospy.get_param("~size_x", 38)
    y = rospy.get_param("~size_y", 60)
    z = rospy.get_param("~size_z", 58)
    #For testing
    # x = rospy.get_param("~size_x", 120)
    # y = rospy.get_param("~size_y", 75)
    # z = rospy.get_param("~size_z", 170)

    #pos in m
    center_pos_x = rospy.get_param("~center_pos_x", 0)
    center_pos_y = rospy.get_param("~center_pos_y", 0.52)
    center_pos_z = rospy.get_param("~center_pos_z", -0.29)

    corner_pos_x = rospy.get_param("~corner_pos_x", -0.3)
    corner_pos_y = rospy.get_param("~corner_pos_y", 0.71)
    corner_pos_z = rospy.get_param("~corner_pos_z", -0.29)

    #robot ur5 max y reach= 85 cm

    ws = workspace(x, y, z, Point(center_pos_x, center_pos_y, center_pos_z), Point(corner_pos_x, corner_pos_y, corner_pos_z)) #Create a new instance of the workspace class
    
    rospy.spin()

if __name__ == '__main__':

    main()
