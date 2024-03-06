<a name="readme-top"></a>
<br />
<div align="center">
<h3 align="center">IPG-ROS Simulation</h3>
<p align="center">
        BCN eMotorsport
  </p>
</div>

<details>
    <summary>Table of Contents</summary>
    <ol>
        <li>
        <a href="#introduction">Introduction</a>
        </li>
        <li><a href="#disclaimer">Disclaimer</a>
        </li>
        <li>
        <a href="#dependencies">Dependencies</a>
        </li>
        <li>
        <a href="#quick-start">Quick Start</a>
        </li>
        <li>
        <a href="#ros1ros2">ROS1/ROS2</a>
        </li>
    </ol>
</details>

# Introduction
This repo holds an IPG CarMaker Formula Student project with the ROS extension, enabling communication between both frameworks. 

Having an IPG simulation platform with a ROS environment enable us to try different parts of the Autonomous Systems pipeline (mostly Autonomous Control) with a fine tunned Multibody Vehicle Model. Thus, a better correlation between simulation and testing results can be achieved, which traduces to an enhanced performance in the real track. Moreover, with IPG a full pipeline simulation is possible, meaning you can run the simulation for EV control algorithms (low level), Autonomous Control (high level) and Perception algorithms (high level).

From experience we know the best way to try (and fine-tune) our perception stack is by using real testing data. For this reason, this simulation includes a fake perception module (more info [here](/ros/ros1_ws/README.md)) so we can simulate our AS Control together with our Low Level Control pipelines keeping the simulation light (without Raw LiDAR pointcloud generation, usually GPU-demanding).

With this project, our team has been able to simulate for the first time our complete Vehicle Controls software stack, which involves:
* AS Control:
    * Lateral NMPC
    * Adaptive Cruise Control
    * Tracklimits
    * Path Planning
* Low Level Control (shared with manual):
    * Regenerative Braking
    * Torque Vectoring
    * Traction Control
    * Power Control

# Disclaimer
If you use this simulation pkgs the **only** thing I ask for is to **ALWAYS REFERENCE** the team ___BCN eMotorsport___.

For obvious reasons, the vehicle model parametrization of our prototype __is not included__ in this repo. The default IPG's FS vehicle model is given. You should specify your own car specs for a precise simulation.

# Dependencies
* [Ubuntu](https://ubuntu.com/) 20.04
* [ROS](https://www.ros.org/) Noetic
* [Eigen3](https://eigen.tuxfamily.org)
* [IPG CarMaker](https://www.ipg-automotive.com/en/products-solutions/software/carmaker/) 11.0.1 license
* [Catkin Tools](https://catkin-tools.readthedocs.io/en/latest/) (optional)

___NOTE:__ I strongly recommend installing [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) for building this project. If not, you must edit the [build.sh](/ros/ros1_ws/build.sh) bash script in order to build the project with the ROS default catkin_make command._

# Quick Start
1. Manually __re-create__ the _libcmcppifloader_ symbolic link to the latest version of the CMRosIF library, which is currently `libcmcppifloader-linux64.so.1.0.0`:
```sh
cd lib/
sudo ln -sfn libcmcppifloader-linux64.so.1.0.0 libcmcppifloader-linux64.so
```

2. Check added paths during the IPG default installation. You should have this commands written down in your '.bashrc' file (_if not copy and paste them at the end of the file_):
```sh
# Additional paths for IPG CM
addpath ()
{ for d in "$@"; do PATH="$PATH:$d"; done; }
addpath /opt/ipg/hil/linux/bin /opt/ipg/hil/linux/GUI /opt/ipg/hil/linux/GUI64
addpath /opt/ipg/bin /opt/ipg/carmaker/linux64/bin /opt/ipg/carmaker/linux64/GUI
```

3. __Build__ the CarMaker Project and ROS workspace:
```sh
cd /path_to/FSDriverless  # go to the repo dir
chmod +x ./build.sh       # only once
./build.sh                
```

4. Finally, __run__ a CarMaker instance (now with CMRosIF extension):
```
./CMStart.sh
```

The loaded TestRun should be __FS_autonomous_Trackdrive__. This TestRun will run the car on an infinite track in order to use IPG only as the physics engine of the simulation (at the lowest layer). 

For more information about the simulation workflow [READ THIS](/ros/ros1_ws/README.md).

## Build command failed

If `./build.sh` command failed, you should open the Car Maker GUI in order to update the project:
```sh
cd /path_to/FSDriverless # go to the repo dir
./CMStart.sh
# NOTE: usually some processes will die or fail after openning the GUI at this point. It's ok, we only want to update our CM Project 
```
Once there, go to `File` > `Project Folder` > `Update Project..` and select the option `Sources / Build Environment`. This command will update the necessary files inside this repo in order to build the CM Project with your computer specs. 

After updating the project, [User.c](src/User.c) and [Makefile](src/Makefile) will have been overwritten, so IPG won't know this repo holds the CMRosIF extension. To fix this, substitude these files with the [User.c](src_cmrosif/User.c) and [Makefile](src_cmrosif/Makefile) files found inside the [src_cmrosif](src_cmrosif/) directory.


# ROS1/ROS2
Within this project we can find two ROS workspaces, one for ROS1 and another for ROS2. The CMRosIf is compatible with both, so it is possible to keep simulating with IPG once your pipeline works with ROS2. However, keep in mind that this whole project has been created in order to work with ROS1 (Noetic), so it is possible that some build flags and/or IPG parameters will have to be changed in order to be compatible with ROS2.
