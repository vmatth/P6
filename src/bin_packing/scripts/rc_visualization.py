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
from bin_packing.convertTo2DArray import convertTo2DArray #convert function


#Class for the packing system. Defines the workspace plots
class packing_visualization:

    #Init function for the packing setup. Creates the workspace & height map plots. Workspace sizes are in [cm]. 
    def __init__(self):
        #self.subscriber = rospy.Subscriber("/packing_info", Packing_info , self.parcel_callback)
        self.subscriber = rospy.Subscriber("/workspace/info", Workspace, self.workspace_callback)
        self.has_initialized = False

        self.show_fig()


        #self.workspace_size = [(x, y, z)] #Save the workspace size in a variable


    def workspace_callback(self, data):
        if self.has_initialized is False:
            return
        print("Receiving data from /workspace/info")
        plt.ion()
        biggest_ax = data.size.x
        if data.size.y > biggest_ax:
            biggest_ax = data.size.y
        if data.size.z > biggest_ax:
            biggest_ax = data.size.z
        #Set figure limits
        self.ax3d.set_xlim([0, biggest_ax])
        self.ax3d.set_ylim([0, biggest_ax])
        self.ax3d.set_zlim([0, biggest_ax])

        self.ax2d.set_xlim([0, biggest_ax])
        self.ax2d.set_ylim([0, biggest_ax])

        # self.ax3d.set_xlim([0, 10])
        # self.ax3d.set_ylim([0, 10])
        # self.ax3d.set_zlim([0, 10])

        # self.ax2d.set_xlim([0, 10])
        # self.ax2d.set_ylim([0, 10])



        #Visualize the parcels
        positions = []
        sizes = []
        colors = []
        for i in range(0, len(data.parcels)):
            positions.append((data.parcels[i].end_pos.x, data.parcels[i].end_pos.y, data.parcels[i].end_pos.z))
            sizes.append((data.parcels[i].actual_size.x, data.parcels[i].actual_size.y, data.parcels[i].actual_size.z))
            colors.append('#3c94ec4d')


        if len(positions) > 0:
            del self.ax3d.collections[:]
            self.parcels = self.plotCubeAt2(positions,sizes,colors=colors, edgecolor="k")
            self.ax3d.add_collection3d(self.parcels) 

        #Height map
        height_map = convertTo2DArray(data.height_map, True)
        self.colorbar.remove()
        c = plt.pcolormesh(height_map, edgecolors='k', linewidths=0.5, cmap='viridis', vmin=0.0, vmax=data.size.z)
        self.colorbar = plt.colorbar(c)
        plt.show()
        plt.pause(0.01)

    #Callback function when a new parcel is published to the /parcel_info topic
    def parcel_callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
        #call add_parcel()
        p = parcel((data.end_pos.x, data.end_pos.y, data.end_pos.z), (data.actual_size.x, data.actual_size.y, data.actual_size.z))
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
    def show_fig(self):
        if self.has_initialized:   
            return
        self.fig = plt.figure(figsize=plt.figaspect(0.5))

        self.ax3d = self.fig.add_subplot(1,2,1, projection='3d')

        self.ax3d.set_xlim([0, 20])
        self.ax3d.set_ylim([0, 20])
        self.ax3d.set_zlim([0, 20])
        self.ax3d.set_title('3D Workspace')
        self.ax3d.set_aspect('equal')
        self.ax3d.set_xlabel('x')
        self.ax3d.set_ylabel('y')
        self.ax3d.set_zlabel('z')

        self.ax2d = self.fig.add_subplot(1,2,2)

        self.ax2d.set_title('2D Heightmap')
        self.ax2d.set_xlabel('x')
        self.ax2d.set_ylabel('y')

        res_x = int(20)
        res_y = int(20)

        self.ax2d.set_xlim([0, 20])
        self.ax2d.set_ylim([0, 20])

        # Create a 2D array using x,y
        #self.height_map_array = [[0 for i in range(res_x)] for j in range(res_y)] 

        w = 12
        h = 5
        l = self.ax2d.figure.subplotpars.left
        r = self.ax2d.figure.subplotpars.right
        t = self.ax2d.figure.subplotpars.top
        b = self.ax2d.figure.subplotpars.bottom
        figw = float(w)/(r-l)
        figh = float(h)/(t-b)
        self.ax2d.figure.set_size_inches(figw, figh)

        # set the spacing between subplots
        plt.subplots_adjust(left=0.1,
                    bottom=0.1, 
                    right=0.9, 
                    top=0.9, 
                    wspace=0.4, 
                    hspace=0.4)

        plt.ion()   
        self.has_initialized = True
        #plt.pcolormesh(c=None,edgecolors='k', linewidths=0.5, cmap='viridis', vmin=0.0, vmax=20)
        c = plt.pcolormesh([[]])
        self.colorbar = plt.colorbar(c)
        plt.show(block=True)
        plt.pause(0.01)  



    # def add_parcel(self, parcel):
    #     print("adding parcel to workspace")

    #     self.parcels.append(parcel) #Add parcel to the workspace

    #     #Visualize the parcel using matplotlib
    #     positions = [parcel.position]        
    #     sizes = [parcel.size]        
    #     colors = ['#3c94ec4d'] #:)
    #     print("colooooooors", colors)
        
    #     pc = self.plotCubeAt2(positions,sizes,colors=colors, edgecolor="k")
    #     self.ax3d.add_collection3d(pc) 

    #     pos_x = int(positions[0][0])
    #     pos_y = int(positions[0][1])
    #     pos_z = int(positions[0][2])
    #     size_x = int(ma.ceil(sizes[0][0]))
    #     size_y = int(ma.ceil(sizes[0][1]))
    #     size_z = int(ma.ceil(sizes[0][2]))

    #     #Add parcel to height map, by changing each pixel (x,y) to the height
    #     for x in range(pos_x, pos_x + size_x):
    #         for y in range(pos_y, pos_y + size_y):
    #             self.height_map_array[y][x] = size_z + pos_z
            

            
    #     plt.ion()
    #     plt.pcolormesh(self.height_map_array,edgecolors='k', linewidths=0.5, cmap='viridis', vmin=0.0, vmax=20)
    #     plt.show()
    #     plt.pause(0.01)
        


def main():
    rospy.init_node('visualization', anonymous=True)

    ps = packing_visualization() 

if __name__ == '__main__':
    main()