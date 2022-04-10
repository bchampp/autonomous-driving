<div align="center">

  <img src="doc/img/qcar.png" alt="logo" width="250" height="auto" />
  <h1>Autonomous Driving using SLAM</h1>
  
  <p>
    An open-source autonomous driving system built with ROS 1. 
  </p>
  
  
<!-- Badges -->
<p>
  <a href="">
    <img src="https://img.shields.io/github/last-commit/bchampp/autonomous-driving" alt="last update" />
  </a>
  <a href="https://github.com/bchampp/autonomous-driving/stargazers">
    <img src="https://img.shields.io/github/stars/bchampp/autonomous-driving" alt="stars" />
  </a>
  <a href="https://github.com/bchampp/autonomous-driving/issues/">
    <img src="https://img.shields.io/github/issues/bchampp/autonomous-driving" alt="open issues" />
  </a>
  <a href="https://github.com/bchampp/autonomous-driving/blob/master/LICENSE">
    <img src="https://img.shields.io/github/license/bchampp/autonomous-driving.svg" alt="license" />
  </a>
</p>
   
<h4>
    <!-- TODO: Insert YouTube Video -->
    <a href="">View Demo</a>
  <span> · </span>
    <a href="https://github.com/bchampp/autonomous-driving/doc/perception.md">Documentation</a>
  <span> · </span>
    <a href="https://github.com/bchampp/autonomous-driving/issues/">Report Bug</a>
  </h4>
</div>

<br />

<!-- Table of Contents -->

# :notebook_with_decorative_cover: Table of Contents

- [About the Project](#star2-about-the-project)
- [Getting Started](#toolbox-getting-started)
- [Usage](#eyes-usage)
- [License](#warning-license)
- [Contact](#handshake-contact)
- [Acknowledgements](#gem-acknowledgements)

<!-- About the Project -->

## :star2: About the Project

This project implements an autonomous driving system for the Quanser QCar, along with a simulation environment to accurately test in.

The modules implemented in this project are:

1. Hardware interface with camera, depth, imu, and motor control.
2. Image processing pipeline to calibrate and synchronize data.
3. Perception system including lane detection and object detection.
4. Planning system to control the cars behaviour.
5. Control system to effectively move the car.
6. Simulation using CARLA and Gazebo.

<!-- Screenshots -->

## :toolbox: Getting Started

This project is implemented using the Robotic Operating System (ROS). The root of the repository acts as the catkin workspace. ROS packages are found in `src/`.

Build the catkin workspace:

```
source /opt/ros/$ROS_DISTRO/setup.bash # melodic or noetic
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 # ensure it is built with python3
source devel/setup.bash # update the environment
```

### Simulation

#### Gazebo

<div align="center"> 
  <img src="doc/img/gazebo.png" alt="screenshot" />
</div>

Gazebo is used as a simulation environment for the system. URDF files are provided to accurately model the vehicle. More instructions for running the gazebo simulation can be found in the `qcar_gazebo` package.

To run the simulation with all systems running:

```
roslaunch qcar_gazebo qcar_world.launch
```

To run the simulation with only perception systems running:

```
roslaunch qcar_gazebo qcar_perception.launch
```

#### CARLA

CARLA is an open source simulation platform built with the Unreal Engine. The perception systems can be run in the CARLA world using:

```
roslaunch qcar_carla qcar_perception
```

<!-- Contact -->

## :handshake: Authors

Brent Champion - brent.champion@queensu.ca
Erin Peterson - erin.peterson@queensu.ca
Laure Halabi - laure.halabi@queensu.ca
Raed Fayad - raed.fayad@queensu.ca

<!-- License -->

## :warning: License

This project is licensed under the MIT license - see the [License](./LICENSE.md) for details.
