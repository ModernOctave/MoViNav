#!/bin/bash

cp -rf ~/XTDrone/sensing/slam/vio/VINS-Fusion ~/catkin_ws/src/ &&\
cp -f ~/XTDrone/sensing/slam/vio/xtdrone_run_vio.sh ~/catkin_ws/scripts &&\
sed -i 's/px4_sitl_stereo_imu_config.yaml/px4_sitl_mono_imu_config.yaml/' ~/catkin_ws/scripts/xtdrone_run_vio.sh &&\
cd ~/catkin_ws &&\
catkin clean &&\
catkin_make