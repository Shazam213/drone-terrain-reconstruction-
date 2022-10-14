# Project
Flying a drone over some terrain in [ROS](http://wiki.ros.org/ROS/Tutorials) with a GPS and a Depth Sensor and to construct a 3D model of that terrain with the incorporation of the Point Cloud Library [(PCL)](http://wiki.ros.org/pcl/Tutorials). <br>

<table>
  <tr>
   <th colspan="4">Flying our drone over a sample terrain:</th>
 </tr>
 <tr>
  <td colspan="4" align="center"><img src = "assets/drone-flying.gif" width="800" height="480" /></td>
 </tr>
 <tr>
    <th colspan="2"> Getting Point Cloud Data of that terrain: </th>
    <th colspan = "2"> And reconstructing it: </th>
  </tr>
  <tr>
    <td colspan = "2"> <img src = "https://user-images.githubusercontent.com/95737452/195854089-5d94c0bd-0c95-446f-a87f-22292b4ac60d.png" width="640" height="300" />  </td>
    <td colspan = "2"> <img src = "https://user-images.githubusercontent.com/95737452/195854646-7a62a1f9-df38-457d-8854-4eeb05b6e146.png" width="640" height="300" />  </td>
 </tr>
</table>
                                                          
              

<!-- TABLE OF CONTENTS -->
## Table Of Contents
* [Project](#project)
  * [Table of Contents](#table-of-contents)
  * [About The Project](#about-the-project)
    * [Tech Stack](#tech-stack)
    * [File Structure](#file-structure)
  * [Getting Started](#getting-started) 
    * [Prerequisites and installlation](#prerequisites-and-installlation)
    * [Installation](#installation)
    * [Execution](#execution)
  * [Process Flow](#process-flow)
  * [Results and Demo](#results-and-demo)
  * [Future Work](#future-work)
  * [Contributors](#contributors)
  * [Acknowledgements and Resources](#acknowledgements-and-resources)
  * [License](#license)

<!-- ABOUT THE PROJECT -->
## About the Project

* The idea is to have a Drone fly over some terrain in ROS with a GPS and a Depth sensor, then get the pointcloud data from the drone and create a 3D map of the topography of the terrain.
* The traditional problem addressed by surface reconstruction is to recover the digital representation of a physical shape that has been scanned, where the scanned data contains a wide variety of defects. At its core, therefore, surface reconstruction is the process by which a 3D object is inferred, or “reconstructed”, from a collection of discrete points that sample the shape, which is in our case, is obtained from LiDAR Sensors. 
* Throughout the course of this project, we learnt about several reconstruction techniques that included such as:
  * Subsampling
  * Upsampling
  * Estimation of surface normals
  * Surface reconstruction from normals.
* The project was started with the use of ROS to obtain the Point Cloud data of a terrain with the use of LiDAR sesnsors. The use of Point Cloud Library followed to create 3D Polygonal Meshes and understanding its usage and experimenting with the various algorithms used in PCL for reconstructing meshes from the point clouds. 
* PCL makes use of many algorithms like Marching Cubes, Grid Projection, Greedy Projection, etc. After experimenting, the project focused on creating the mesh using the Greedy Projection Triangulation algorithm. The data for GPT has been created from the Point Cloud using Moving Least Squares via Maximum Likelihood Estimation.

### Tech Stack
* [ROS Noetic](http://wiki.ros.org/noetic)
* [Gazebo](http://gazebosim.org/)
* [RViz](http://wiki.ros.org/rviz/UserGuide)
* [Point Cloud Library](https://pcl.readthedocs.io/en/latest/)
* [MeshLab](https://www.meshlab.net/#download) (optional)

### File Structure
```
📦Drone-3D-topography
 ┣ 📂assets                           #contains gifs, videos and images of the results
 ┣ 📂config                           #Rviz config files
 ┣ 📂include                          #include files for the plugins
 ┃ ┗ 📜DialogKeyboard.h  
 ┃ ┗ 📜drone_object_ros.h             
 ┃ ┗ 📜pid_controller.h             
 ┃ ┗ 📜plugin_drone.h             
 ┃ ┗ 📜plugin_ros_cam.h             
 ┃ ┗ 📜sensor_model.h
 ┃ ┗ 📜util_ros_cam.h                 
 ┣ 📂launch                           #launch files
 ┃ ┗ 📜simple.launch
 ┣ 📂models                           #files and meshes used to render the model                       
 ┃ ┗ 📂kinect
 ┃ ┃ ┣ 📂materials
 ┃ ┃ ┣ 📂meshes
 ┃ ┃ ┣ 📜model.config
 ┃ ┃ ┗ 📜model.sdf
 ┣ 📂plugins                         #plugins for the model
 ┃ ┗ 📜libplugin_drone.so
 ┃ ┗ 📜libplugin_ros_cam.so                   
 ┣ 📂scripts                          #C++ program used to run the drone
 ┃ ┣ listener.cpp                     #Used to get PCD from the drone and to process the data using PCL
 ┣ 📂src                              #contains custom plugins used with the drone
 ┃ ┣ 📜DialogKeyboard.cpp
 ┃ ┗ 📜DialogKeyboard.ui
 ┃ ┗ 📜drone_keyboard.cpp
 ┃ ┗ 📜drone_object_ros.cpppid_controller.cpp
 ┃ ┗ 📜plugin_drone.cpp
 ┃ ┗ 📜plugin_ros_cam.cpp
 ┃ ┗ 📜plugin_ros_init.cpp
 ┃ ┗ 📜util_ros_cam.cpp
 ┣ 📂urdf
 ┃ ┗ 📜sjtu_drone.urdf
 ┣ 📂worlds                           #world files
 ┃ ┣ 📜terrain.world
 ┣ 📜CMakeLists.txt
 ┣ 📜README.md
 ┗ 📜package.xml
 
 ```
<!-- GETTING STARTED --> 
 ## Getting Started
 
 ### Prerequisites and Installlation
 
* Tested on [Ubuntu 20.04](https://ubuntu.com/download/desktop)
* [ROS Noetic](http://wiki.ros.org/noetic/Installation)
* [Gazebo Sim](http://gazebosim.org/)
* [RViz](http://wiki.ros.org/rviz/UserGuide)
* [Point Cloud Library](https://pointclouds.org/downloads/#linux)

 
### Installation

```sh
git clone https://github.com/Shazam213/Drone-3d-topography.git
```
Add this folder to the **src** directory of your **catkin workspace**. If you haven't yet created the src folder, do so using the following command:

```sh
mkdir src
```
Initialise the project with

```sh
catkin build
source ~/catkin_ws/devel/setup.bash
```

### Execution

Open three terminals and run the following commands:

* Terminal 1:                                         

```sh
source ~/catkin_ws/devel/setup.bash
roslaunch sjtu_drone simple.launch
```

* Terminal 2:                                         
```sh
source ~/catkin_ws/devel/setup.bash
rosrun sjtu_drone drone_keyboard
```

* Terminal 3:

```sh
source ~/catkin_ws/devel/setup.bash
rosrun sjtu_drone listener
```
<!-- PROCESS FLOW -->
## Process Flow
<img src = "https://user-images.githubusercontent.com/95737452/195857258-1affa14d-25fd-453d-8de0-b0970d3c3111.png" height="500" width="1000">

<img src = "https://user-images.githubusercontent.com/95737452/193408478-136fde79-c182-48d3-b425-07b2550f21b1.png" height="400" width="1000">


<!-- RESULTS AND DEMO -->
## Results and Demo

https://user-images.githubusercontent.com/95737452/195919753-824d5377-6222-4f1e-95f2-8006cc685aa8.mp4




<!-- FUTURE WORK -->
## Future Work
- [ ] Implement other surface reconstruction algorithms like [Marching Cubes Algorithm](https://pointclouds.org/documentation/classpcl_1_1_marching_cubes.html) and [Poisson Surface Reconstruction](https://pointclouds.org/documentation/classpcl_1_1_poisson.html) using PCL.
- [ ] Uderstand [CGAL](https://www.cgal.org/) To implement other surface reconstruction algorithms.
- [ ] Understand the implementation of Delaunay Triangulation using CGAL.
- [ ] Improve the algorithm to make it work for multiple frames of data.

<!-- CONTRIBUTORS -->
## Contributors
* [Soham Mulye](https://github.com/shazam213)
* [Unmani Shinde](https://github.com/unmani-shinde)

<!-- ACKNOWLEDGMENTS AND RESOURCES -->
## Acknowledgements and Resources
* [SRA VJTI](http://sra.vjti.info/) Eklavya 2022  
* [Danping Zou & Tahsincan Köse](https://github.com/tahsinkose/sjtu-drone) for the model of the drone. 
* [Navpreet Kaur Pawar](https://nccastaff.bmth.ac.uk/jmacey/OldWeb/MastersProjects/MSc13/14/Thesis/SurfaceReconstructionThesis.pdf) for the master thesis 'Surface Reconstruction from Point Clouds', which was extremely illuminating.
* Our mentors [Jash Shah](https://github.com/Jash-Shah) and [Sarrah Bastawala](https://github.com/sarrah-basta) for their guidance throughout the whole project.

<!-- LICENSE -->
## License
[MIT License](https://opensource.org/licenses/MIT)
