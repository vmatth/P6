#!/usr/bin/env python
from turtle import pos
import rospy
from read_camera.msg import Parcel
import matplotlib as mlp
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import math as ma
from bin_packing.msg import Packing_info
from bin_packing.parcel import parcel
from bin_packing.msg import Workspace #workspace msg


#Class for the packing system. Defines the workspace plots
class packing_visualization:

    #Init function for the packing setup. Creates the workspace & height map plots. Workspace sizes are in [cm]. 
    def __init__(self, x, y, z):
        self.subscriber = rospy.Subscriber("/packing_info", Packing_info , self.parcel_callback)

        self.parcels = [] #Stores all of the parcels in the workspace

        self.setup_workspace(x, y, z)
        self.setup_height_map(x, y, z)

        self.workspace_size = [(x, y, z)] #Save the workspace size in a variable

        #self.subscriber = rospy.Subscriber("/workspace", Workspace, self.workspace_callback)


    #Callback function when a new parcel is published to the /parcel_info topic
    def parcel_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        #call add_parcel()
        p = parcel((data.pos.x, data.pos.y, data.pos.z), (data.size.x, data.size.y, data.size.z))
        self.add_parcel(p)
        

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

        w = 12
        h = 5
        l = self.ax2d.figure.subplotpars.left
        r = self.ax2d.figure.subplotpars.right
        t = self.ax2d.figure.subplotpars.top
        b = self.ax2d.figure.subplotpars.bottom
        figw = float(w)/(r-l)
        figh = float(h)/(t-b)
        self.ax2d.figure.set_size_inches(figw, figh)



        plt.ion()   
        c = plt.pcolormesh(self.height_map_array,edgecolors='k', linewidths=0.5, cmap='viridis', vmin=0.0, vmax=z)
        plt.colorbar(c)
        plt.show(block=True)
        plt.pause(0.01)
    
    def add_parcel(self, parcel):
        print("adding parcel to workspace")

        self.parcels.append(parcel) #Add parcel to the workspace

        #Visualize the parcel using matplotlib
        positions = [parcel.position]        
        sizes = [parcel.size]        
        colors = ['#3c94ec4d'] #:)
        print("colooooooors", colors)
        
        pc = self.plotCubeAt2(positions,sizes,colors=colors, edgecolor="k")
        self.ax3d.add_collection3d(pc) 

        pos_x = int(positions[0][0])
        pos_y = int(positions[0][1])
        pos_z = int(positions[0][2])
        size_x = int(ma.ceil(sizes[0][0]))
        size_y = int(ma.ceil(sizes[0][1]))
        size_z = int(ma.ceil(sizes[0][2]))

        #Add parcel to height map, by changing each pixel (x,y) to the height
        for x in range(pos_x, pos_x + size_x):
            for y in range(pos_y, pos_y + size_y):
                self.height_map_array[y][x] = size_z + pos_z
            

            
        plt.ion()
        plt.pcolormesh(self.height_map_array,edgecolors='k', linewidths=0.5, cmap='viridis', vmin=0.0, vmax=20)
        plt.show()
        plt.pause(0.01)
        


def main():
    rospy.init_node('packing_visualization', anonymous=True)

    ps = packing_visualization(20, 20, 20) #Size in cm

    #p3 = parcel((2,4,8), (3, 6, 9))    

    # p3 = parcel((1,6,7), (2, 4, 8))    
    # ps.add_parcel(p3)

    # p3 = parcel((4,4,3), (8, 6, 3))    
    # ps.add_parcel(p3)

    # p3 = parcel((5,10,11), (12, 8, 4))    
    # ps.add_parcel(p3)

    # p3 = parcel((11,2,3), (5, 10, 15))    
    # ps.add_parcel(p3)






if __name__ == '__main__':
    main()