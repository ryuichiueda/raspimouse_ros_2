#!/bin/bash -xve

#sync and make
rsync -av ./ ~/catkin_ws/src/raspimouse_ros_2/
cd ~/catkin_ws
catkin_make || catkin_make
