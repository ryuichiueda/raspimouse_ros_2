[![Build Status](https://travis-ci.org/ryuichiueda/raspimouse_ros_2.svg?branch=master)](https://travis-ci.org/ryuichiueda/raspimouse_ros_2)

# raspimouse_ros_2

The current version of the ROS base package for Raspberry Pi Mouse. This package is derived from "pimouse_ros" package, which is coded for the book from Nikkei BP.
* old versions: [ryuichiueda/raspimouse_ros](https://github.com/ryuichiueda/raspimouse_ros)

## Requirements

This package requires the following to run:

* Ubuntu
  * Ubuntu 16.04
  * Ubuntu MATE 16.04
* ROS 
  * ROS Kinetic Kame
* Device Driver
  * [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse)

## Installation

### 1. Install the latest stable version of ROS.  

Please refer to [ROS WiKi](http://wiki.ros.org/kinetic/Installation) for installation, or run the following ros setup scripts.
* [ryuichiueda/ros_setup_scripts_Ubuntu16.04_server](https://github.com/ryuichiueda/ros_setup_scripts_Ubuntu16.04_server)

### 2. Download and install device driver.  

Please refer to [rt-net/RaspberryPiMouse](https://github.com/rt-net/RaspberryPiMouse) for download and installation.

### 3. Make a workspace.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_worksapce
cd ~/catkin_ws
catkin_make
```

Delete the following line from `~/.bashrc`:

```
source /opt/ros/kinetic/setup.bash
```

Add the follwing line to `~/.bashrc`:

```
source ~/catkin_ws/devel/setup.bash
```

### 4. Download this repository into `~/catkin_ws/src`.

```
cd ~/catkin_ws/src
git clone https://github.com/ryuichiueda/raspimouse_ros_2.git
```

### 5. Resolve the system dependencies.

```
rosdep install raspimouse_ros_2
```

### 6. Build this repository.

```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```


### 7. Test with the buzzer node

```
roslaunch raspimouse_ros_2 raspimouse.launch
rostopic pub /buzzer std_msgs/UInt16 1000
```

## Support of RT-USB-9axisIMU2

If you have an RT-USB-9axisIMU2 9-axis sensor, its z angular velocity can be reflected to the `/odom` topic. 


### requirement

* hardware: RT-USB-9axisIMU2 (text mode)
* software: https://github.com/AtsushiSaito/rt_usb_9axis_sensor

### configuration

Please set `1` to the argument `imu`. When you want to use this feature permanently, please rewrite the arg element of raspimouse.launch as follows. 

```raspimouse.launch 
<launch>
  <arg name="imu" default="1" />   <!-- change from 0 to 1 -->
    <include if="$(arg imu)" file="$(find rt_usb_9axis_sensor)/launch/rt_usb_9axis_sensor.launch" />
...
```
