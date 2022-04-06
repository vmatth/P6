#!/usr/bin/env python
import rospy
from bin_packing.msg import Workspace #workspace msg
from bin_packing.parcel import parcel #parcel class
from geometry_msgs.msg import Point #point type
from bin_packing.msg import Packing_info
from bin_packing.msg import Height_Map_Row
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout

class workspace:
    def __init__ (self, x, y, z):
        self.parcels = []
        self.workspace_size = Point(x,y,z)

        self.height_map = [[0.0 for i in range(x)] for j in range(y)] 
        self.sub = rospy.Subscriber("/workspace/add_parcel", Packing_info, self.add_parcel)
        self.pub = rospy.Publisher("/workspace", Workspace, queue_size=10)

        self.update_workspace()

        print("Creating new workspace with size. ", self.workspace_size)

    def add_parcel(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Workspace received new parcel %s", data)
        #create a parcel class
        self.parcels.append(data)

        #Add parcel to height map, by changing each pixel (x,y) to the height
        for x in range(int(data.pos.x), int(data.pos.x) + int(data.size.x)):
            for y in range(int(data.pos.y), int(data.pos.y) + int(data.size.y)):
                self.height_map_array[x][y] = int(data.size.z) + int(data.pos.z)
            
        self.update_workspace()
        
    def update_workspace(self):
        msg = Workspace()
        msg.size = self.workspace_size

        hm = []

        for y in range(0, self.workspace_size.y):
            hm_row = Height_Map_Row()
            hm_row.row_data = self.height_map[y]
            #print("rm rooooow", hm_row)
            hm.append(hm_row)

        msg.height_map = hm
        msg.parcels = self.parcels
        self.pub.publish(msg)


def main():
    rospy.init_node('workspace', anonymous=True)
    #rosrun pkg node _x:=2 _y:5 _z:=10
    x = rospy.get_param("~x", 20)
    y = rospy.get_param("~y", 20)
    z = rospy.get_param("~z", 20)

    ws = workspace(x, y, z) #Create a new instance of the workspace class
    
    rospy.spin()

if __name__ == '__main__':

    main()
