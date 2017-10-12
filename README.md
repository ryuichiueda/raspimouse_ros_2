[![Build Status](https://travis-ci.org/ryuichiueda/raspimouse_ros_2.svg?branch=dev)](https://travis-ci.org/ryuichiueda/raspimouse_ros_2)

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


## Usage

* Sound buzzer

    ```
    roslaunch raspimouse_ros_2 raspimouse.launch
    rostopic pub /buzzer std_msgs/UInt16 1000
    ```

* Control motor

    ```
    echo 1 > /dev/rtmotoren0
    roslaunch raspimouse_ros_2 raspimouse.launch
    rostopic pub /motor_raw raspimouse_ros_2/MotorFreqs "left_hz: 400
    right_hz: 400"
    ```
