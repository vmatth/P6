# P6 Robotics Bachelor Project

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

### Test Kinect2 (If you receive USB errors try to restart your computer)

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

Navigate to Setup Robot / Setup Network
Check Static Address
Input a reasonable IP address, e.g. 172.22.22.2
Input Netmask as 255.255.255.0, Gateway and DNS server as 0.0.0.0

Setup wired PC network:

    Choose static IP address, e.g 172.22.22.1

Add the computer's IP address to the UR5 using the controller

Connect the UR5 and PC using an Ethernet cable

