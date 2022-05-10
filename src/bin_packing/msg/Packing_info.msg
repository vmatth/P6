
#Actual size is the size measured by the camera.
#This size is given in cm and contains decimals.
geometry_msgs/Point actual_size

#Rounded size is the size in integers (whole numbers).
#It is used for the packing algorithm as it does not support decimals.
geometry_msgs/Point rounded_size

#The size without any rotations.
#This is needed as actual_size is rotated by the packing algorithm
#However, the robot needs the non-rotated-size when picking up the parcel
geometry_msgs/Point non_rotated_size

#Start position for picking
geometry_msgs/Point start_pos

#End position for packing
geometry_msgs/Point end_pos

#The angle of the parcel detected by the camera
float32 angle
#1-3: The packing algorithm tells us where to pick up the parcel to rotate it
int8 picking_side
#Amount of rotation for the packing part. Either 0 or 90
int64 parcel_rotation
