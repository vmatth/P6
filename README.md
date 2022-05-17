# P6 Robotics Bachelor Project

Overview of the ROS nodes:
![alt text](src/nodes.png?raw=true "Nodes" width="100")

This README explains how to build the workspace and run the nodes.
## Setup ROS Melodic
On Ubuntu 18.04 download ROS Melodic 
http://wiki.ros.org/melodic/Installation/Ubuntu


## Setup Kinect2 Camera

### Install libfreenect2 to your HOME directory (Copy everything line by line)
```
cd ~/
git clone https://github.com/OpenKinect/libfreenect2.git
cd libfreenect2

sudo apt-get install build-essential cmake pkg-config

sudo apt-get install libusb-1.0-0-dev

sudo apt-get install libturbojpeg0-dev

sudo apt-get update
sudo apt-get install libglfw3-dev

sudo apt-get install beignet-dev
```

Clone freenect2-python to your HOME directory
```
cd ~/
git clone https://github.com/rjw57/freenect2-python
```

Add the following lines to the ~/.bashrc file at HOME
```
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$HOME/freenect2/lib/pkgconfig
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/freenect2/lib
```
Close and open the terminal for the .bashrc files to have an effect

cd to libfreenect and build
```
cd ~/libfreenect2
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2
make
make install
```

### Install CUDA (If you have nvidia)
https://developer.nvidia.com/cuda-downloads

After following the installation add the following lines to the ~/.bashrc file
```
export LD_LIBRARY_PATH="/usr/local/cuda/lib64:${LD_LIBRARY_PATH}"
export PATH="/usr/local/cuda/bin:${PATH}"
```
A system-wide configuration of the libary path can be created with the following commands
```
echo "/usr/local/cuda/lib64" | sudo tee /etc/ld.so.conf.d/cuda.conf
sudo ldconfig
```

### Test Kinect2 (Only works on USB 3.0)

Connect Kinect2 and test the camera using libfreenect2:

```
cd ~/libfreenect2/build/bin
./Protonect gl
./Protonect cpu
```
If you have CUDA set up:
```
./Protonect cuda
```

### Install dependencies and build the workspace
```
cd ~/P6
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y
catkin_make
```
### Source P6 workspace in ~/.bashrc (Restart your terminal after for bashrc to take effect)
```
source ~/P6/devel/setup.bash
```

## Using the Kinect2 camera in ros (This command must always run when using Kinect2 with ros)
Run the following command to allow ros to use Kinect2 data
```
roslaunch kinect2_bridge kinect2_bridge.launch
```

## Setup UR5 Robot

### Setup UR5 network:

Connect the UR5 and PC using an Ethernet cable

Using the UR5 controller:

  Navigate to Setup Robot / Setup Network

  Check Static Address

  Input a reasonable IP address, e.g. 172.22.22.2

  Input Netmask as 255.255.255.0, Gateway and DNS server as 0.0.0.0

### Setup wired PC network:

Choose static IP address, e.g 172.22.22.1

Add the computer's IP address to the UR5 using the controller

## Import module errors when running python scripts
If the following error occurs:
```
No module named imageio
```
Download imageio version 2.6.1 as a ZIP file at

https://github.com/imageio/imageio/tree/v2.6.1

Extract the files and install the module. (At the imageio folder directory)
```
sudo python setup.py install
```

## Running all the nodes

Run each command in a new terminal
### Bin packing nodes
```
roslaunch bin_packing bin_packing_bring_up.launch 
```
### Robot Nodes (Physical Robot)
```
roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=172.22.22.2 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml
```
```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch
```
```
roslaunch ur5_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5_moveit_config)/launch/moveit.rviz
```
```
rosrun robot_mover mover.py
```
### Robot Nodes (Simulation)
```
roslaunch ur_gazebo ur5_bringup.launch limited:=true
```
```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true
```
```
roslaunch ur5_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5_moveit_config)/launch/moveit.rviz
```
```
rosrun robot_mover mover.py _sim:=True
```
### Camera Nodes
```
roslaunch kinect2_bridge kinect2_bridge.launch 
```
```
rosrun camera_trigger trigger.py 
```




