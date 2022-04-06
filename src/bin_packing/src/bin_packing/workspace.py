#!/usr/bin/env python
import rospy

class workspace:

    #Init function for the packing setup. Creates the workspace & height map plots. Workspace sizes are in [cm]. 
    def __init__(self, x, y, z):
        #rospy.Subscriber("/parcel_info", Parcel , self.parcel_callback)

        self.parcels = [] #Stores all of the parcels in the workspace

        self.workspace_size = (x, y, z) #Save the workspace size in a variable

        self.height_map_array = [[0 for i in range(x)] for j in range(y)] 
