#!/bin/bash

cp -r ~/XTDrone/motion_planning/3d/ego_planner ~/catkin_ws/src/ &&\
cd ~/catkin_ws/ &&\
catkin build ego_planner