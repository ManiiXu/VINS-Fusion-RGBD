# VINS-Fusion for RGB-D

This is a specific verision of VINS-Fusion( based on https://github.com/HKUST-Aerial-Robotics/VINS-Fusion ) that supports RGB-D sensors. Some codes we reference the project VINS-RGBD (https://github.com/STAR-Center/VINS-RGBD)

## 一种可支持RGB-D传感器的VINS-Fusion算法

1、可支持单目、双目、RGB-D与IMU的融合定位  
RGBD在视觉跟踪时将特征点的参数化与双目相机一致，可实现静止状态的初始化。  

2、使用RGB-D时可实现基于八叉树的稠密建图  
实时生成局部三维稠密地图，地图可在图优化后进行修正，可保存关键帧的位姿图信息（带有三维稠密点云地图的），
可读取位姿图信息载入先前构建的三维地图，可通过载入地图的方式实现多地图融合。
  
3、支持栅格图导入  
栅格图的生成在support_files中，主要利用保存的位姿图实现。  

实验中使用realsense d435i传感器，相关参数参考realsense_depth_imu_config.yaml。

## 1. Prerequisites

### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **OpenCV** 
OpenCV 2.4.8

### 1.3. **Ceres Solver** 
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.4. **PCL** 
PCL 1.7 

## 2. Build 
```
    cd ~/catkin_ws/src
    git clone https://github.com/ManiiXu/VINS-Fusion.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

## 3. Example 

### 3.1 运行局部定位  
rosrun vins vins_node /home/xuduo/catkin_ws/src/VINS-Fusion/config/realsense/realsense_depth_imu_config.yaml

### 3.2 运行全局定位与稠密建图  
rosrun loop_fusion loop_fusion_node  /home/xuduo/catkin_ws/src/VINS-Fusion/config/realsense/realsense_depth_imu_config.yaml

### 3.3 录制bag文件  
rosbag record /camera/imu /camera/color/image_raw /camera/aligned_depth_to_color/image_raw

仅供学习。