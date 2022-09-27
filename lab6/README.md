# Installation

1. Add new packages to your catkin workspace using wstool:
```
cd {VNAV_HOME}/vnav_ws/src/
cp -r {VNAV_HOME}/Labs/Lab_6 ./
wstool merge {VNAV_HOME}/vnav_ws/src/Lab_6/install/lab_6.rosinstall -y
wstool update -j8
```

> **Note** this might take 5min, opengv is large.

4. Build lab_6:
```
catkin build lab_6
```

Remember to source your workspace:
```
source {VNAV_HOME}/vnav_ws/devel/setup.bash
```

# Usage

1. To run the pose estimation on the given rosbag, we will use:
```
roslaunch lab_6 video_tracking.launch
```
> **Note** Not so fast, first you have to implement the functions inside the source code, follow the handout at this point.

