# Autonomous Driving using SLAM

This project implements a prototype of an autonomous vehicle on the Quanser QCar.

<p align="center">
    <img width="300px" src="./doc/img/qcar.png" />
</p>

## Goals
The main goal of this project is to learn about the different components that make up a self driving car stack. This project explores different approaches to perception, mapping, planning and control systems. 

The modules implemented in this project are:
1. Hardware interface with camera, depth, imu, and motor control.
2. Image processing pipeline to calibrate and synchronize data.
2. Perception system including lane detection and object detection.
3. Planning system to control the cars behaviour.
4. Control system to effectively move the car.
5. Simulation using CARLA and Gazebo.

## Getting Started
This project is implemented using the Robotic Operating System (ROS). 

Before building the project, you must first install the vision_opencv package that is approriate for your ROS version from this git repo (the default version is Noetic): https://github.com/ros-perception/vision_opencv/tree/noetic

To download the vision_opencv library:
```
cd ros/src
git clone -b <ROS_version> git@github.com:ros-perception/vision_opencv.git
```

Now you can build your project. The catkin workspace is stored in the `ros` folder. So we must change to our parent directory and run a few commands:
```
cd ..
source /opt/ros/<ROS_Version>/setup.bash # The QCar uses melodic, everywhere else we use noetic
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

## Data Collection and Playback
Launch files have been included to easily collect and replay data against the developed system. These files are included in the `qcar` package as `collect.launch` and `playback.launch`. The collect script will collect data from all relevant sensors for the system using the controller as input. The playback script will replay a rosbag and run all the systems against the recorded data.

To collect data:
```
roslaunch qcar collect.launch
```

To replay data:
```
roslaunch qcar playback.launch
```

## Hardware Interface
The hardware interface contains nodes to interact with the hardware available on the car. This is implemented in the `qcar` package in the ros workspace. 

The following nodes are implemented:
- `commandnode.py` - This is an interface to interact with a Logitech F710 controller
- `csinode.py` - This is an interface to interact with the fisheye cameras on the car
- `lidarnode.py` - This is an interface to interact with the RP Lidar.
- `rgbdnode.py` - This is an interface to interact with the Intel Realsense D435 camera
- `qcarnode.py` - This is an interface to interact with the motors on the car

Launch files have been included in the qcar package to connect to these sensors. More detail can be found in the package documentation. 

To run the vehicle and sensors manually using the controller:
```
roslaunch qcar manual_control.launch
```
<b>Note:</b> You may get an error running this on a fresh QCar setup as the QCar libraries require root permissions to actuate the hardware controllers. In this case, update the sudoers file and add the following line at the end: 
```
nvidia ALL=(ALL) NOPASSWD: ALL
```
This will allow ros to execute sudo without requiring a password. 

## Perception
The perception interface is implemented in the `qcar_perception` package.

The following nodes are implemented: 
- `object_detection.py` - This node detects objects using a YOLOv3 neural network 
- `lane_detection.py` - This node detects lane lines using computer vision techniques

## Planning
The planning system is implemented in the `qcar_planning` package.


## Simulation
### Gazebo
Gazebo is an open source simulation platform developed by the same team as ROS. The team used Gazebo to model accurate environments to test the autonomous driving software in. This was used heavily for developing how the perception and planning systems interact. 

<p align="center">
    <img width="800px" src="./doc/img/gazebo.png" />
</p>

More specific information for running the gazebo simulation can be found in `qcar_gazebo`.

To run the simulation with all systems running:
```
roslaunch qcar_gazebo qcar_world.launch
```

To run the simulation with only perception systems running:
```
roslaunch qcar_gazebo qcar_perception.launch
```

## CARLA Integration
CARLA is an open source autonomous driving simulator built on the Unreal Engine. CARLA was used to test the accuracy of the perception systems implemented, and extend the project beyond the QCar. 

To run the CARLA simulation with the perception systems running:
```
roslaunch qcar_carla qcar_perception
```
