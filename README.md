
<!-- PROJECT LOGO -->
<br />
<p align="center">

  <h3 align="center">Android Vehicle State Estimation (avse)</h3>

  <p align="center">
    Using the android_sensors_driver app, subscribes to key information and publishes esimated state. (WIP)
    <br />
    <a href="https://wiki.ros.org/android_sensors_driver">android_sensors_driver »</a>
    <br />
  </p>
</p>



<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
  * [Built With](#built-with)
* [Getting Started](#getting-started)
  * [Requiements](#requirements)
  * [Get the App](#get-the-app)
  * [Installation](#installation)


<!-- ABOUT THE PROJECT -->
## About The Project
N/A, WIP


### Built With

* []()
* []()
* []()



<!-- GETTING STARTED -->
## Getting Started

### Requirements

* An android phone
* The android_sensors_drivers app
* ROS melodic

### Get the App

* Download the android_sensors_driver app onto your phone from the Wiki page
```sh
http://wiki.ros.org/android_sensors_driver
```
* Follow the tutorial to connect via roscore
```sh
http://wiki.ros.org/android_sensors_driver/Tutorials/Connecting%20to%20a%20ROS%20Master
```
### Installation
0. Install 'sudo apt install ros-melodic-desktop-full' and 'sudo apt install python-catkin-tools'
1. Clone the repo to an existing ROS Workspace (use catkin build)
```sh
git clone https://github.com/mfkeane/avse.git
```
2. Clone common_msgs repo to the same ROS Workspace
```sh
git clone https://github.com/ros/common_msgs.git
```
3. Build the ROS Workspace with catkin build
4. Source the ROS Workspace
```sh
source devel/setup.bash
```
5. From the ROS Workspace:
```sh
rosrun avse gpslistener.py
```

