#!/bin/bash

cp -r ~/XTDrone/motion_planning/3d/ego_planner ~/catkin_ws/src/ &&\
cd ~/catkin_ws/ &&\
sudo apt-get install -y ros-melodic-pcl-ros &&\
catkin clean &&\
catkin_make