# P6 Robotics Bachelor Project

## Setup ROS Melodic
On Ubuntu 18.04 download ROS Melodic 
http://wiki.ros.org/melodic/Installation/Ubuntu


## Setup Kinect2 Camera

### Install libfreenect2 to your HOME directory
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

cd to libfreenect and build
```
cd ~/libfreenect2
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -Dfreenect2_DIR=$HOME/freenect2/lib/cmake/freenect2
make
make install
```

### Install CUDA
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

### Test Kinect2

Connect Kinect2 and test the camera using libfreenect2:

```
cd ~/libfreenect2/build/bin
./Protonect gl
./Protonect cpu
```

### Build the workspace
```
cd ~/P6
catkin_make
```
### Test the kinect in ros
```
roslaunch kinect2_bridge kinect2_bridge.launch
```
