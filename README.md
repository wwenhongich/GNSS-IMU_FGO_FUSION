# GNSS/IMU fusion based on the factor graph

**GNSS/IMU fusion based on the factor graph.**


## Menu

  - [**System architecture**](#system-architecture)

  - [**Package dependency**](#dependency)

  - [**Package install**](#install)

  - [**Sample datasets**](#sample-daasets)

  - [**Run the package**](#run-the-package)

## System architecture

 

## Dependency


## Install

Use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone wwenhwwenhongich/GNSS-IMU_FGO_FUSION.git
cd ..
catkin_make
```

## Sample datasets


## Run the package

1. Run the launch file:
```
roslaunch gnss_imu_fusion run.launch
```

2. Play existing bag files:
```
rosbag play your-bag.bag 
```

