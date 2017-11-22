# image_labeling
Image Labeling package for AtlasCarV2 project
=============================================
This package run under [ROS Kinetic](http://wiki.ros.org/kinetic) on Ubuntu 16.04.

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
git clone https://github.com/nmssilva/image_labeling.git
```
- Install dependencies
```bash
sudo apt-get install libraw1394-11 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0 libpcap-dev libpcap0.8-dev
```
- Compile
```bash
catkin_make -C ~/catkin_ws
```
Testing the package (playback bag)
====
- Download the test [bagfile](https://drive.google.com/open?id=1f4wrlu4qNZCrphA6dpfiomSeSiRELuUR)
- Extract it to the src folder
```bash
tar -xf bags.tar.gz -C ~/catkin_ws/src/image_labeling/image_labeling/
```
- Source the workspace
```bash
source ~/catkin/devel/setup.bash
```
- Launch playback
```bash
roslaunch image_labeling playback.launch
Usage: roslaunch image_labeling playback.launch bags:=<path/to/bagfile> (default path is path to test.bag)
```
Acquire data on the car
====
- [Download](https://www.ptgrey.com/support/downloads) and Install Flycapture SDK for Ubuntu 16.04 (need to register at ptgrey.com) 
- Turn on car
- Turn on sensors in the power box
- Connect ethernet cable
- Disable Wi-Fi
- Set Manual IP address to 192.168.0.1
- Launch get_data
```bash
roslaunch image_labeling get_data.launch
Usage: roslaunch image_labeling get_data.launch record:=<false default, true to record bagfile>
```
