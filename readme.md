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

To start MoViNav,
```
roslaunch monodepth vins_octo_mono.launch
```

To start MoViNav with octomap,
```
roslaunch monodepth map.launch
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
To take off press the 'i' key util upwards vel is higher that 0.3. Then press 'b' followed by 't' to takeoff. Press 'k' to hover.

# Use Ego Planner
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


# Troubleshooting
## Build space was previously built by catkin build / catkin_make.
```
catkin clean
```