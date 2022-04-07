#!/usr/bin/env python
import rospy
import numpy as np

#it just works
def convertTo2DArray(input, invert):
    size_x = len(input)
    output = [[0 for i in range(len(input[0].row_data))] for j in range(len(input))] 
    for x in range(0, size_x):
        for y in range(0, len(output[x])):
            output[x][y] = input[x].row_data[y]
    if(invert):
        output = np.array(output).T.tolist()
        print("flipped", output)
    print(type(output))
    return output