# Autonomous Driving using SLAM

This is my final capstone project, an autonomous driving car using SLAM.

## Goals

The main goal of this project is to learn about the different components that make up a self driving car stack.

The modules implemented in this project are:

1. Sensor interfaces with camera, depth and IMU data.
2. Perception system including lane detection and object detection.
3. Planning system to control the cars behaviour.
4. Control system to effectively move the car.
5. Simulation using CARLA and Simulink.

## Getting Started
Navigate to the ros folder:
`cd ros`

The first step in running a ROS application is sourcing the correct ROS version:

`source /opt/ros/melodic/setup.bash`

You can check the available ROS packages with this command:
`rospack list-names`

Next, we need to build the packages that we're interested in deploying:
`catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`.

This will build the packages defined in our `ros/src/`



