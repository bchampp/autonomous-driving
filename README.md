# ROS Information

Here you will find all the information about the ROS packages, nodes and topics that were used in this project.

## Packages

### QCar

The purpose of the `qcar` package is to implement the nodes required for the sensors and actuators of the car.

#### Cameras

Nodes:

- `rgbdnode` - This is the Intel Realsense camera.
- `csinode` - This is the fisheye cameras mounted on every side of the car.

Publishes:

- `/qcar/realsense_color`
- `/qcar/realsense_depth`
- `/qcar/csi_front`

#### Car sensors

Node:

- `qcarnode` - This is the main node of the car for actuation.

Publishes:

- `/qcar/imu`
- `/qcar/velocity`

Subscribes:

- `/control`

### Perception

The `perception` package performs various computer vision tasks from incoming camera data. This data is then published to appropriate topics to control the cars action.

#### Object Detection

Nodes:

- `objectdetection` - This node implements an object detection system.

Subscribes:

- `/qcar/realsense_color`
- `/qcar/realsense_depth`

Publishes:

- `/perception/object_detections`
- `/perception/visualize_objects`

#### Lane Detection

Nodes:

- `lanedetection` - This node implements a lane detection system.

Subscribes:

- `/qcar/csi_front`

Publishes:

- `/perception/lane_detections`
- `/perception/visualize_lanes`

### Planning

The planning package is used to implement a basic planning system for the cars movement.

Nodes:

- `planningnode` - This node implements a basic planning system for the car actuation

Subscribes:

- `/perception/object_detections`
- `/perception/lane_detections`

Publishes:

- `/qcar/motors`

### Simulation

The purpose of the `simulation` package is to connect the system to the CARLA and Simulink simulation platforms.

For more information on the simulation package, refer to the documentation here.

Nodes:

- `carla_client`
- `simulated_camera`

Launch Files:

- `carla_client.launch`
- `carla_client_rviz.launch`
- `carla_client_rqt.launch`
- `simulated_camera.launch`
