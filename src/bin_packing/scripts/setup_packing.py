#!/usr/bin/env python
from turtle import pos
import rospy
from read_camera.msg import Parcel
import matplotlib.pyplot as plt
import matplotlib as mlp
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import math as ma

#Class for a new individual parcel. Contains information about the parcel's position and parcel's size.
class parcel:
    def __init__(self, position, size):
        self.position = position
        self.size = size

    def parcel_info(self):
        return ("Parcel: | position: ", self.position, " | size ", self.size)

#Class for the packing system. Defines the workspace plots
class packing_setup:

    #Init function for the packing setup. Creates the workspace & height map plots. Workspace sizes are in [cm]. 
    def __init__(self, x, y, z):
        rospy.Subscriber("/parcel_info", Parcel , self.parcel_callback)

        self.parcels = [] #Stores all of the parcels in the workspace

        self.setup_workspace(x, y, z)
        self.setup_height_map(x, y, z)

        self.workspace_size = [(x, y, z)] #Save the workspace size in a variable


    #Callback function when a new parcel is published to the /parcel_info topic
    def parcel_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

    def cuboid_data2(self,o, size=(1,1,1)):
        X = [[[0, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]],
            [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]],
            [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
            [[0, 0, 1], [0, 0, 0], [0, 1, 0], [0, 1, 1]],
            [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
            [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]]]
        X = np.array(X).astype(float)
        for i in range(3):
            X[:,:,i] *= size[i]
        X += np.array(o)
        return X

    def plotCubeAt2(self,positions,sizes=None,colors=None, **kwargs):
        if not isinstance(colors,(list,np.ndarray)): colors=["C0"]*len(positions)
        if not isinstance(sizes,(list,np.ndarray)): sizes=[(1,1,1)]*len(positions)
        g = []
        for p,s,c in zip(positions,sizes,colors):
            g.append( self.cuboid_data2(p, size=s) )
        return Poly3DCollection(np.concatenate(g), facecolors=np.repeat(colors,6), **kwargs)
    
    #Define the 3D workspace for bin packing. Arguments are x,y,z sizes for the workspace
    def setup_workspace(self, x, y, z):
        self.fig = plt.figure(figsize=plt.figaspect(0.5))

        self.ax3d = self.fig.add_subplot(1,2,1, projection='3d')

        self.ax3d.set_xlim([0, x])
        self.ax3d.set_ylim([0, y])
        self.ax3d.set_zlim([0, z])
        self.ax3d.set_title('3D Workspace')
        self.ax3d.set_aspect('equal')
        self.ax3d.set_xlabel('x')
        self.ax3d.set_ylabel('y')
        self.ax3d.set_zlabel('z')

        # set the spacing between subplots
        plt.subplots_adjust(left=0.1,
                    bottom=0.1, 
                    right=0.9, 
                    top=0.9, 
                    wspace=0.4, 
                    hspace=0.4)

        plt.ion()
        plt.show()
        plt.pause(0.01)  

    #Define the height map for bin packing. Arguments are x,y sizes, and res_x, res_y is the resolution.
    def setup_height_map(self, x, y, z):
        self.ax2d = self.fig.add_subplot(1,2,2)

        self.ax2d.set_title('2D Heightmap')
        self.ax2d.set_xlabel('x')
        self.ax2d.set_ylabel('y')

        res_x = int(x)
        res_y = int(y)

        print("Res_x:", )
        print("Res_y:", y)


        self.ax2d.set_xlim([0,x])
        self.ax2d.set_ylim([0,y])

        # Create a 2D array using x,y
        # Creates a list containing 5 lists, each of 8 items, all set to 0
        self.height_map_array = [[0 for i in range(res_x)] for j in range(res_y)] 


        plt.ion()   
        c = plt.pcolormesh(self.height_map_array,edgecolors='k', linewidths=0.5, cmap='viridis', vmin=0.0, vmax=z)
        plt.colorbar(c)
        plt.show
        plt.pause(0.01)
    
    def add_parcel(self, parcel):
        print("adding parcel to workspace")

        self.parcels.append(parcel) #Add parcel to the workspace

        #Visualize the parcel using matplotlib
        positions = [parcel.position]        
        sizes = [parcel.size]        
        colors = ['navy'] #:)
        
        pc = self.plotCubeAt2(positions,sizes,colors=colors, edgecolor="k")
        self.ax3d.add_collection3d(pc) 

        #positions[0][0] = pos_x
        #positions[0][1] = pos_y

        #Add parcel to height map, by changing each pixel (x,y) to the height
        for x in range(positions[0][0],positions[0][0]+int(ma.ceil(sizes[0][0]))):
            for y in range(positions[0][1],positions[0][1]+int(ma.ceil(sizes[0][1]))):
                self.height_map_array[y][x] = int(ma.ceil(sizes[0][2]))
            
            
        plt.ion()
        plt.pcolormesh(self.height_map_array,edgecolors='k', linewidths=0.5, cmap='viridis', vmin=0.0, vmax=self.workspace_size[0][2])
        plt.show()
        plt.pause(0.01)
        


def main():
    ps = packing_setup(20, 20, 20) #Size in cm

    p = parcel((0,0,0), (9.5, 9.2, 20))    
    ps.add_parcel(p)

    p2 = parcel((10,15,0), (3, 4, 10))    
    ps.add_parcel(p2)

    p3 = parcel((4,8,12), (1, 1, 5))    
    ps.add_parcel(p3)

    # p3 = parcel((2,4,8), (3, 6, 9))    
    # ps.add_parcel(p3)

    # p3 = parcel((1,6,7), (2, 4, 8))    
    # ps.add_parcel(p3)

    # p3 = parcel((4,4,3), (8, 6, 3))    
    # ps.add_parcel(p3)

    # p3 = parcel((5,10,11), (12, 8, 4))    
    # ps.add_parcel(p3)

    # p3 = parcel((11,2,3), (5, 10, 15))    
    # ps.add_parcel(p3)

    print("All parcels in workspace: ")
    for x in ps.parcels:
        print(x.parcel_info())

    rospy.init_node('packing_setup', anonymous=True)
    rospy.spin()





if __name__ == '__main__':
    main()