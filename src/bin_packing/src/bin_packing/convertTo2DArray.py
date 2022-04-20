#!/usr/bin/env python
import rospy
import numpy as np
from bin_packing.msg import Height_Map_Row

#it just works
def convertTo2DArray(input, invert):
    size_x = len(input)
    output = [[0 for i in range(len(input[0].row_data))] for j in range(len(input))] 
    for x in range(0, size_x):
        for y in range(0, len(output[x])):
            output[x][y] = input[x].row_data[y]
    if(invert):
        output = np.array(output).T.tolist()
        #print("flipped", output)
    #print(type(output))
    return output

def convertToMultiArray(input, size_x):
    output = []
    for x in range(0, int(size_x)):
        row = Height_Map_Row()
        row.row_data = input[x]
        output.append(row)
    return output
