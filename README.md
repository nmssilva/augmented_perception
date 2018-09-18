# ⚠ Instructions incomplete due to missing material ⚠

Augmented Perception package for the ATLASCAR2
==============================================
This package was built under [ROS Kinetic](http://wiki.ros.org/kinetic) on Ubuntu 16.04.

Setup
=====

- Create a workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```
- Clone this repository into the src folder
```bash
cd ~/catkin_ws/src
git clone https://github.com/nmssilva/augmented_perception.git
```
- Install dependency libraries
```bash
sudo apt-get install libraw1394-11 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0 libpcap-dev libpcap0.8-dev
```
- Fetch [depency packages](https://linkmissing.mec.ua.pt) (missing link) ⚠
- Compile
```bash
catkin_make -C ~/catkin_ws
```
Testing the package (playback bag)
==================================
- Download the test [bagfiles](https://linkmissing.mec.ua.pt) (missing link) ⚠
- Extract it to the src folder
```bash
tar -xf bags.tar.gz -C ~/catkin_ws/src/image_labeling/image_labeling/
```
- Source the workspace
```bash
source ~/catkin/devel/setup.bash
```

Acquire data on the car
=======================
- [Download](https://www.ptgrey.com/support/downloads) and Install Flycapture SDK for Ubuntu 16.04 (need to register at ptgrey.com) 
- Turn on car
- Turn on sensors in the power box
- Connect ethernet cable
- Disable Wi-Fi
- Set Manual IP address to 192.168.0.1
- Launch the drivers
```bash
roslaunch image_labeling drivers.launch
```
- Camera IP mut be set manually using the ```flycap``` command 
- Devices are ready to be used. Use Rviz or any tool to visualize laser and camera data.
