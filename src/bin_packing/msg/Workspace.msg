# Size of the workspace in cm
geometry_msgs/Point size

# Position of the workspace's center relative to the robot frame
geometry_msgs/Point center_position
geometry_msgs/Point corner_position

Height_Map_Row[] height_map
Packing_info[] parcels

#Used to remove a specified number of parcels from the workspace
#By giving -1 to this message, all of the parcels will be removed
int8 parcels_to_yeet

#True or false if the data should be saved to a csv file
bool save_data