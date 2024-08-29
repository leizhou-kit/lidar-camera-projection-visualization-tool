# lidar camera projection visualization tool

This tool is used to visualize the fusion of point cloud and camera image after external parameter calibration.


##Usage

### 1. Prerequisites

Ubuntu, ROS, PCL, OpenCV.


### 2. Run

```
mkdir -p catkin_ws/src
cd catkin_ws/src
```
Download this code to your local path.
```
git clone https://github.com/leizhou-kit/lidar-camera-projection-visualization-tool.git
```

```
cd ..
catkin_make
source ~/devel/setup.bash
```

after ajust your own parameters in config file, you can now run the code with

```
roslaunch urban_projection projection.launch
```


