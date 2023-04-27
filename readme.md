# Setup MoViNAV
See [Setup Instructions](/setup.md)

# Launch
Start the simulation
```
roslaunch px4 indoor1-depth.launch
```

Start VINS-Fusion
```
source ~/catkin_ws/devel/setup.bash
bash ~/catkin_ws/scripts/xtdrone_run_vio.sh
```

Start Octomap
```
roslaunch monodepth vins_octo_mono.launch
```

Change coordinate system direction of camera pose
```
python ~/XTDrone/motion_planning/3d/ego_transfer.py iris 0
```

Start rviz
```
rviz -d ~/XTDrone/motion_planning/3d/ego_rviz.rviz
```

Start ego_planner
```
roslaunch ego_planner single_uav.launch 
```

# Control Manually
Establish a connection to the drone
```
cd ~/XTDrone/communication
python multirotor_communication.py iris 0 
```

Use keyboard to control the drone
```
cd ~/XTDrone/control/keyboard
python multirotor_keyboard_control.py iris 1 vel
```

## Take off


# Troubleshooting
## Build space was previously built by catkin build / catkin_make.
```
catkin clean
```