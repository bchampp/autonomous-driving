# QCar Gazebo
This package implements the qcar integration with the gazebo simulation platform. 

## Worlds
Gazebo worlds are defined in the `/worlds` directory. Some sample worlds have been included, but the best world for testing the qcar in is the `qcar.world` file. 

## Models
Models are included in the `meshes`, `urdf` and `model_` folders. The QCar modelling is done in the meshes and urdf folders, where the 3D CAD files are included and then put together in the URDF language. 

<b>Note:</b> For the Intel RealSense to work with the qcar, additional dependencies are included in the `realsense_gazebo_plugin` and `realsense2_description` packages. This plugin allows us to simulate the data being streamed for the realsense realistically. 

