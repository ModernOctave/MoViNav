# XTDrone Docker
## Install NVIDIA Driver
```sudo ubuntu-driver autoinstall```

## Install NVIDIA Docker
[Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

## Get Docker Image
Download docker image from [here](https://drive.google.com/file/d/19ToxmvjeOwSZznAMA-2XUPFPec58uzTR/view?usp=sharing).

Then import it into docker using the following command,
```
docker load -i xtdrone_1_4.tar.gz
```

The above procedure provides a modified version v1.4 of the docker image. The modified version uses global repositories instead of the chinese mirrors available in the original XTDrone image. 

The original image can be found [here](https://www.yuque.com/xtdrone/manual_en/docker#a73vU).

## Create Container
```
docker run --runtime=nvidia --name MoViNav -it -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY -e XAUTHORITY -e NVIDIA_DRIVER_CAPABILITIES=all xtdrone:1.4
```

## Clone repository
```	
git clone https://github.com/ModernOctave/MoViNav.git ~/MoViNav
cd ~/MoViNav
```

## Install Ceres Solver
```
./install-ceres.sh
```

## Install VINS-Fusion
```
./install-vins.sh
```

## Change Camera
Create a copy of ~/PX4_Firmware/launch/indoor1.launch.

```
cp ~/PX4_Firmware/launch/indoor1.launch ~/PX4_Firmware/launch/indoor1-depth.launch
```

Change the camera from `iris_stereo_camera` to `iris_realsense_camera` so that ground truth depth data can be seen.

```
sed -i 's/iris_stereo_camera/iris_realsense_camera/g' ~/PX4_Firmware/launch/indoor1-depth.launch
```

## Install ego_planner
```
./install-ego-planner.sh
```

## Install MoViNav
```
./install-movinav.sh
```