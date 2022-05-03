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

class workspace:
    def __init__ (self, x, y, z):
        self.parcels = []
        self.workspace_size = Point(x,y,z)

        self.height_map = [[0.0 for i in range(y)] for j in range(x)] 
        self.sub = rospy.Subscriber("/workspace/add_parcel", Packing_info, self.add_parcel)
        self.pub = rospy.Publisher("/workspace/info", Workspace, queue_size=10, latch=True)
        self.sub = rospy.Subscriber("/workspace/update_height_map", Workspace, self.heightmap_callback)

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
        msg = Workspace()
        msg.size = self.workspace_size

        hm = convertToMultiArray(self.height_map, self.workspace_size.x)

        msg.height_map = hm
        msg.parcels = self.parcels
        self.pub.publish(msg)

        parcelsV = []
        for i in range(len(self.parcels)):
            parcel_volume = self.parcels[i].rounded_size.x * self.parcels[i].rounded_size.y * self.parcels[i].rounded_size.z 
            parcelsV.append(parcel_volume)
        
        #print("Parcel_volume: ", parcel_volume)
        #print("list of volumes: ", parcelsV)
        parcels_combined_volume = sum(parcelsV)
        print("Combined Volume: ", parcels_combined_volume)
        v_workspace = self.workspace_size.x * self.workspace_size.y * self.workspace_size.z
        print("Workspace Volume: ", v_workspace)
        if parcels_combined_volume > 0:
            fill_rate = int(parcels_combined_volume / v_workspace  * 100)
            print("fill_rate: ", fill_rate, "%")
            num_parcels = len(parcelsV)
            print("Parcel nr.: ", num_parcels)

        #loop all parcels
            #calculate volume for each parcel
            #add the volumes together
        #find volume
            #calculate the workspace volume

        #calculate fill-rate V_parcels/V_workspace *100

    def heightmap_callback(self, data):
        print("Height map updated with size: ", data.size)
        print("receivin hm", data.height_map)
        self.workspace_size = data.size
        self.height_map = convertTo2DArray(data.height_map, False)
        self.update_workspace()


def main():
    rospy.init_node('workspace', anonymous=True)
    #rosrun pkg node _x:=2 _y:5 _z:=10
    x = rospy.get_param("~x", 50)
    y = rospy.get_param("~y", 80)
    z = rospy.get_param("~z", 30)

    ws = workspace(x, y, z) #Create a new instance of the workspace class
    
    # rate = rospy.Rate(1) # 10hz
    # while not rospy.is_shutdown():
    #     ws.update_workspace()
    #     rate.sleep()

    rospy.spin()

if __name__ == '__main__':

    main()
