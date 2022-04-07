#!/usr/bin/env python
import rospy
from bin_packing.msg import Workspace #workspace msg
from bin_packing.parcel import parcel #parcel class
from geometry_msgs.msg import Point #point type
from bin_packing.msg import Packing_info #Packing msg
from bin_packing.msg import Height_Map_Row
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout

class workspace:
    def __init__ (self, x, y, z):
        self.parcels = []
        self.workspace_size = Point(x,y,z)

        self.height_map = [[0.0 for i in range(y)] for j in range(x)] 
        self.sub = rospy.Subscriber("/workspace/add_parcel", Packing_info, self.add_parcel)
        self.pub = rospy.Publisher("/workspace/info", Workspace, queue_size=10, latch=True)

        self.update_workspace()

        print("Creating new workspace with size. ", self.workspace_size)

    def add_parcel(self, data):
        rospy.loginfo(rospy.get_caller_id() + "Workspace received new parcel %s", data)
        #create a parcel class
        self.parcels.append(data)

        #Add parcel to height map, by changing each pixel (x,y) to the height
        for x in range(int(data.pos.x), int(data.pos.x) + int(data.size.x)):
            for y in range(int(data.pos.y), int(data.pos.y) + int(data.size.y)):
                self.height_map[x][y] = int(data.size.z) + int(data.pos.z)
            
        self.update_workspace()
        
    def update_workspace(self):
        msg = Workspace()
        msg.size = self.workspace_size

        hm = []


        for x in range(0, self.workspace_size.x):
            hm_row = Height_Map_Row()
            hm_row.row_data = self.height_map[x]
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
    
    # rate = rospy.Rate(1) # 10hz
    # while not rospy.is_shutdown():
    #     ws.update_workspace()
    #     rate.sleep()

    rospy.spin()

if __name__ == '__main__':

    main()
