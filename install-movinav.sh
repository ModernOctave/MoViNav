#!/bin/bash

cp -rf src/* ~/catkin_ws/src/ &&\
cd ~/catkin_ws &&\
catkin_make