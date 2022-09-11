# GNSS/IMU fusion

**GNSS/IMU loosely coupled fusion based on the factor graph. A video of the result can be found on [YouTube](https://youtu.be/xn7mffXxrPo).We collect real GNSS and IMU on the Xiamen University campus.**
/nAuthors: Wenhong Wu, Xunyu Zhong, Xiaozhen Cui, and Dongjie Wu 

## Menu

  - [**System architecture**](#system-architecture)

  - [**Package dependency**](#dependency)

  - [**Package install**](#install)
  
  - [**Gazebo simulation**](#Gazebo_simulation)

  - [**Run the package**](#run-the-package)

## System architecture
   There are still many bugs in this project.

## Dependency

- [gtsam](https://gtsam.org/get_started/) (Georgia Tech Smoothing and Mapping library)
  ```
  sudo add-apt-repository ppa:borglab/gtsam-release-4.0
  sudo apt update
  sudo apt install libgtsam-dev libgtsam-unstable-dev
  ```

## Install

Use the following commands to download and compile the package.

```
cd ~/catkin_ws/src
git clone wwenhwwenhongich/GNSS-IMU_FGO_FUSION.git
cd ..
catkin_make
```

## Gazebo_simulation


## Run the package

1. Run the launch file:
```
roslaunch gnss_imu_fusion run.launch
```

2. Play existing bag files:
```
rosbag play your-bag.bag 
```

